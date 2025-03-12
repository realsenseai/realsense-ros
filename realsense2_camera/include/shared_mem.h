#ifndef SHARED_MEM_H
#define SHARED_MEM_H

#include <string>
#include <cstddef>


class SharedMem {
public:
    // Constructor: Opens (and optionally creates) a shared memory segment.
    // Throws std::runtime_error on failure.
    SharedMem(const std::string &name, size_t size, bool create);

    // Destructor: Unmaps and closes the shared memory.
    ~SharedMem();

    // Optionally release the resources early.
    // If unmapMem is true, the memory mapping will be unmapped.
    void release(bool unmapMem = true);

    // Returns a pointer to the shared memory.
    char* data() const;

    // Returns the size of the shared memory segment.
    size_t size() const;

    // Returns the name of the shared memory segment.
    std::string name() const;

    // Disable copy semantics.
    SharedMem(const SharedMem&) = delete;
    SharedMem& operator=(const SharedMem&) = delete;

private:
    std::string m_Filename; ///< Name of the shared memory object.
    size_t      m_Size;     ///< Size of the shared memory segment.
    int         m_fMem;     ///< File descriptor for the shared memory.
    char       *m_pMem;     ///< Pointer to the mapped memory.
    bool        m_init;     ///< Flag indicating if initialization succeeded.
};

#endif // SHARED_MEM_H
