#pragma once
#include <string>
#include <stdexcept>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include<cstdint>

class MMapFile 
{
private:
     int fd;
     size_t file_size;
     void* mapped_data;

public:
     MMapFile(const std::string& filepath);

     ~MMapFile();
 
     // Disable copying to prevent double-unmapping
     MMapFile(const MMapFile&) = delete;
     MMapFile& operator=(const MMapFile&) = delete;

      // Provide access to the raw byte pointer
     const uint8_t* data() const;
     size_t size() const;
};