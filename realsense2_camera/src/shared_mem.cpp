#include "shared_mem.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdexcept>
#include <cstring>
#include <iostream>


// the shared memory is opened and mapped
// during construction, and all resources are released in the destructor.
SharedMem::SharedMem(const std::string &name, size_t size, bool create)
    : m_Filename(name), m_Size(size), m_fMem(-1), m_pMem(nullptr), m_init(false)
{
    int flags = O_RDWR | (create ? O_CREAT : 0);
    std::cout << "Creating/Opening (" << create << ") shm file name" << name.c_str() << " with size " << size << " bytes" << std::endl;

    m_fMem = shm_open(m_Filename.c_str(), flags, 0660);
    if (m_fMem == -1) {
        throw std::runtime_error("shm_open failed: " + std::string(std::strerror(errno)));
    }

    // Set size.
    if (ftruncate(m_fMem, m_Size) == -1) {
        close(m_fMem);
        throw std::runtime_error("ftruncate failed: " + std::string(std::strerror(errno)));
    }

    // Map address space.
    m_pMem = static_cast<char*>(mmap(nullptr, m_Size, PROT_READ | PROT_WRITE, MAP_SHARED, m_fMem, 0));
    if (m_pMem == MAP_FAILED) {
        close(m_fMem);
        throw std::runtime_error("mmap failed: " + std::string(std::strerror(errno)));
    }

    m_init = true;
}

SharedMem::~SharedMem() {
    // Unmap the memory
    std::cout << "Closing shm file name" << this->name().c_str() << " with size " << this->size() << " bytes" << std::endl;

    if (m_pMem && m_pMem != MAP_FAILED) {
        munmap(m_pMem, m_Size);
        m_pMem = nullptr;
    }
    // Close the shared memory file
    if (m_fMem != -1) {
        close(m_fMem);
        m_fMem = -1;
    }
}

// manually release the shared memory using release().
void SharedMem::release(bool unmapMem) {
    if (m_fMem != -1) {
        close(m_fMem);
        m_fMem = -1;
    }
    if (unmapMem && m_pMem && m_pMem != MAP_FAILED) {
        munmap(m_pMem, m_Size);
        m_pMem = nullptr;
    }
    m_init = false;
}

// Returns a pointer to the mapped shared memory.
char* SharedMem::data() const {
    return m_pMem;
}

// Returns the size of the shared memory segment.
size_t SharedMem::size() const {
    return m_Size;
}

// Returns the name of the shared memory segment.
std::string SharedMem::name() const {
    return m_Filename;
}
