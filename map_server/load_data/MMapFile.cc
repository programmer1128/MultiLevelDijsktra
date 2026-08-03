#include "MMapFile.hpp"

MMapFile::MMapFile(const std::string& filepath) : fd(-1), file_size(0), mapped_data(MAP_FAILED) 
{
    fd = open(filepath.c_str(), O_RDONLY);
    if (fd == -1) 
    {
        throw std::runtime_error("MMap failed to open file: " + filepath);
    }

    struct stat sb;
    if (fstat(fd, &sb) == -1) 
    {
        close(fd);
        throw std::runtime_error("MMap failed to get file size: " + filepath);
    }
    file_size = sb.st_size;

    //map the file directly into Read-Only memory. since the memory is read only, concurrency is
    //very simple as the threads will read from the same rom space avoiding duplicate memories
    mapped_data = mmap(nullptr, file_size, PROT_READ, MAP_SHARED, fd, 0);
    if (mapped_data == MAP_FAILED) {
        close(fd);
        throw std::runtime_error("MMap failed to map memory: " + filepath);
    }
}

MMapFile::~MMapFile() 
{
    if (mapped_data != MAP_FAILED) 
    {
        munmap(mapped_data, file_size);
    }
    if (fd != -1) 
    {
        close(fd);
    }
}

// Provide access to the raw byte pointer
const uint8_t* MMapFile::data() const 
{
    return static_cast<const uint8_t*>(mapped_data);
} 

size_t MMapFile:: size() const 
{
    return file_size;
}
 