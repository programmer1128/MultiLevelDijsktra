import duckdb
import pandas as pd
import numpy as np
import json

def build_overture_pipeline():
    print("Connecting to Overture Maps on AWS S3 via DuckDB...")
    
    con = duckdb.connect()
    con.execute("INSTALL httpfs; LOAD httpfs;")
    con.execute("INSTALL spatial; LOAD spatial;")
    con.execute("SET s3_region='us-west-2';") 
    
    # India bounding box
    min_lon, min_lat = 68.1, 6.7
    max_lon, max_lat = 97.4, 35.5

    query = f"""
        SELECT 
            'overture' AS source,
            id AS source_id,
            names.primary AS name,
            categories.primary AS category,
            CAST(confidence * 100 AS INTEGER) AS importance_score,
            phones,
            websites,
            socials,
            ST_AsText(geometry) AS geom
        FROM read_parquet('s3://overturemaps-us-west-2/release/2026-07-22.0/theme=places/*/*', filename=true, hive_partitioning=1)
        WHERE bbox.xmin > {min_lon} AND bbox.xmax < {max_lon}
          AND bbox.ymin > {min_lat} AND bbox.ymax < {max_lat}
          AND names.primary IS NOT NULL
    """
    
    print("Executing spatial query across cloud parquet files...")
    df = con.execute(query).df()
    
    print(f"Extraction complete! Found {len(df)} commercial POIs in India.")
    
    # Helper function to convert ndarray or list to standard python list
    def clean_array(val):
        if val is None:
            return None
        if isinstance(val, (list, np.ndarray)):
            if len(val) > 0:
                return val.tolist() if isinstance(val, np.ndarray) else val
        return None

    print("Formatting metadata for 5.2M rows (optimized)...")
    
    phones_arr = df['phones'].to_numpy()
    websites_arr = df['websites'].to_numpy()
    socials_arr = df['socials'].to_numpy()
    
    metadata_list = []
    
    # Fast iteration over 5.2M rows using zip
    for p, w, s in zip(phones_arr, websites_arr, socials_arr):
        meta = {}
        
        clean_p = clean_array(p)
        if clean_p: meta['phones'] = clean_p
        
        clean_w = clean_array(w)
        if clean_w: meta['websites'] = clean_w
        
        clean_s = clean_array(s)
        if clean_s: meta['socials'] = clean_s
        
        metadata_list.append(json.dumps(meta))
        
    df['metadata'] = metadata_list
    
    # Drop intermediate columns
    df = df.drop(columns=['phones', 'websites', 'socials'])
    
    # Match PostgreSQL schema order exactly
    df = df[['source', 'source_id', 'name', 'category', 'importance_score', 'metadata', 'geom']]
    
    print("Writing to overture_locations.csv...")
    df.to_csv('overture_locations.csv', index=False, quoting=1) 
    print("Done! CSV file is ready for database bulk copy.")

if __name__ == "__main__":
    build_overture_pipeline()