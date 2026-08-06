#include <iostream>
#include <fstream>
#include <string>
#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/visitor.hpp>
#include <osmium/geom/wkt.hpp>
#include <nlohmann/json.hpp>

using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

class POIHandler : public osmium::handler::Handler {
    std::ofstream& csv_file;
    osmium::geom::WKTFactory<> wkt_factory; // Generates LineStrings and Polygons

    // Escape strings for CSV (converts " to "")
    std::string escape_csv(const std::string& input) {
        std::string output = input;
        size_t pos = 0;
        while ((pos = output.find("\"", pos)) != std::string::npos) {
            output.replace(pos, 1, "\"\"");
            pos += 2;
        }
        return output;
    }

    std::string build_metadata_json(const osmium::TagList& tags) {
        nlohmann::json j;
        for (const auto& tag : tags) {
            j[tag.key()] = tag.value();
        }
        return escape_csv(j.dump());
    }

    std::string get_primary_category(const osmium::TagList& tags) {
        if (tags.has_key("highway")) return "highway";
        if (tags.has_key("amenity")) return "amenity";
        if (tags.has_key("shop")) return "shop";
        if (tags.has_key("healthcare")) return "healthcare";
        if (tags.has_key("tourism")) return "tourism";
        if (tags.has_key("leisure")) return "leisure";
        if (tags.has_key("historic")) return "historic";
        if (tags.has_key("railway")) return "railway";
        if (tags.has_key("aeroway")) return "aeroway";
        if (tags.has_key("office")) return "office";
        if (tags.has_key("building")) return "building";
        return tags.begin()->key(); // Fallback
    }

    int calculate_importance(const osmium::TagList& tags) {
        // High priority for major infrastructure
        if (tags.has_key("aeroway")) return 100;
        if (tags.has_key("railway")) return 90;
        if (tags.has_key("healthcare")) return 85;
        
        // Dynamic scoring for roads based on size
        if (const char* h = tags.get_value_by_key("highway")) {
            std::string hw(h);
            if (hw == "motorway" || hw == "trunk") return 95;
            if (hw == "primary") return 80;
            if (hw == "secondary") return 70;
            if (hw == "residential") return 40;
            return 30; // lanes, paths, etc.
        }
        return 20; // Default POI
    }

    void write_csv(const std::string& source, const std::string& id, const std::string& name, 
                   const std::string& category, int score, const std::string& json, const std::string& geom) {
        csv_file << source << "," << id << ",\"" << escape_csv(name) << "\",\"" << category << "\"," 
                 << score << ",\"" << json << "\",\"" << geom << "\"\n";
    }

public:
    POIHandler(std::ofstream& out) : csv_file(out) {}

    // Process Points (ATMs, individual shops, etc)
    void node(const osmium::Node& node) {
        if (!node.tags().has_key("name")) return; // Only save named places

        try {
            std::string source_id = "node/" + std::to_string(node.id());
            std::string name = node.tags().get_value_by_key("name");
            std::string geom = "SRID=4326;" + wkt_factory.create_point(node);

            write_csv("osm", source_id, name, get_primary_category(node.tags()), 
                      calculate_importance(node.tags()), build_metadata_json(node.tags()), geom);
        } catch (const osmium::geometry_error&) {
            // Ignore corrupted coordinates
        }
    }

    // Process Lines (Roads) and Polygons (Buildings, Parks)
    void way(const osmium::Way& way) {
        if (!way.tags().has_key("name")) return; // Only save named roads/places

        try {
            std::string source_id = "way/" + std::to_string(way.id());
            std::string name = way.tags().get_value_by_key("name");
            std::string geom;

            // If it is a closed shape AND it is not a roundabout/road, extract as POLYGON
            if (way.is_closed() && !way.tags().has_key("highway")) {
                geom = "SRID=4326;" + wkt_factory.create_polygon(way);
            } 
            // Otherwise, it is a road, lane, or unclosed path. Extract as LINESTRING
            else {
                geom = "SRID=4326;" + wkt_factory.create_linestring(way);
            }

            write_csv("osm", source_id, name, get_primary_category(way.tags()), 
                      calculate_importance(way.tags()), build_metadata_json(way.tags()), geom);
        } catch (const osmium::geometry_error&) {
            // Ignore incomplete ways (e.g., missing node coordinates)
        }
    }
};

int main(int argc, char* argv[]) {
    std::ofstream csv_file("osm_locations_full.csv");
    csv_file << "source,source_id,name,category,importance_score,metadata,geom\n"; // CSV Header

    osmium::io::File input_file{"/home/aritra/MultiLevelDijsktra/parsing/india-260724.osm.pbf"};
    
    index_type index;
    location_handler_type location_handler{index};
    location_handler.ignore_errors(); 
    
    POIHandler poi_handler{csv_file};

    // Note: We are reading nodes and ways. 
    osmium::io::Reader reader{input_file, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
    osmium::apply(reader, location_handler, poi_handler);
    reader.close();

    std::cout << "Data extraction complete. Check osm_locations_full.csv" << std::endl;
    return 0;
}