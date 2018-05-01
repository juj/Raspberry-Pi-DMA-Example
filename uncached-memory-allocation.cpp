#include <sys/mman.h> //for mmap
#include <stdio.h> //for printf
#include <stdlib.h> //for exit
#include <fcntl.h> //for file opening
#include <stdint.h> //for uint32_t
#include <string.h> //for memset
#include <errno.h>
#include <sys/types.h>
#include <unistd.h>

#define PAGE_SIZE 4096 //mmap maps pages of memory, so we must give it multiples of this size

size_t ceilToPage(size_t size) {
    //round up to nearest page-size multiple
    if (size & (PAGE_SIZE-1)) {
        size += PAGE_SIZE - (size & (PAGE_SIZE-1));
    }
    return size;
}

uintptr_t virtToPhys(void* virt, int pagemapfd) {
    uintptr_t pgNum = (uintptr_t)(virt)/PAGE_SIZE;
    int byteOffsetFromPage = (uintptr_t)(virt)%PAGE_SIZE;
    uint64_t physPage;
    ///proc/self/pagemap is a uint64_t array where the index represents the virtual page number and the value at that index represents the physical page number.
    //So if virtual address is 0x1000000, read the value at *array* index 0x1000000/PAGE_SIZE and multiply that by PAGE_SIZE to get the physical address.
    //because files are bytestreams, one must explicitly multiply each byte index by 8 to treat it as a uint64_t array.
    int err = lseek(pagemapfd, pgNum*8, SEEK_SET);
    if (err != pgNum*8) {
        printf("WARNING: virtToPhys %p failed to seek (expected %i got %i. errno: %i)\n", virt, pgNum*8, err, errno);
    }
    read(pagemapfd, &physPage, 8);
    if (!(physPage & (1ull<<63))) { //bit 63 is set to 1 if the page is present in ram
        printf("WARNING: virtToPhys %p has no physical address\n", virt);
    }
    physPage = physPage & ~(0x1ffull << 55); //bits 55-63 are flags.
    uintptr_t mapped = (uintptr_t)(physPage*PAGE_SIZE + byteOffsetFromPage);
    return mapped;
}

uintptr_t virtToUncachedPhys(void *virt, int pagemapfd) {
    return virtToPhys(virt, pagemapfd) | 0x40000000; //bus address of the ram is 0x40000000. With this binary-or, writes to the returned address will bypass the CPU (L1) cache, but not the L2 cache. 0xc0000000 should be the base address if L2 must also be bypassed. However, the DMA engine is aware of L2 cache - just not the L1 cache (source: http://en.wikibooks.org/wiki/Aros/Platforms/Arm_Raspberry_Pi_support#Framebuffer )
}


//allocate some memory and lock it so that its physical address will never change
void* makeLockedMem(size_t size) {
    //void* mem = valloc(size); //memory returned by valloc is not zero'd
    size = ceilToPage(size);
    void *mem = mmap(
        NULL,   //let kernel place memory where it wants
        size,   //length
        PROT_WRITE | PROT_READ, //ask for read and write permissions to memory
        MAP_SHARED | 
        MAP_ANONYMOUS | //no underlying file; initialize to 0
        MAP_NORESERVE | //don't reserve swap space
        MAP_LOCKED, //lock into *virtual* ram. Physical ram may still change!
        -1, // File descriptor
    0); //no offset into file (file doesn't exist).
    if (mem == MAP_FAILED) {
        printf("makeLockedMem failed\n");
        exit(1);
    }
    memset(mem, 0, size); //simultaneously zero the pages and force them into memory
    mlock(mem, size);
    return mem;
}

void* makeUncachedMemView(void* virtaddr, size_t bytes, int memfd, int pagemapfd) {
    //by default, writing to any virtual address will go through the CPU cache.
    //this function will return a pointer that behaves the same as virtaddr, but bypasses the CPU L1 cache (note that because of this, the returned pointer and original pointer should not be used in conjunction, else cache-related inconsistencies will arise)
    //Note: The original memory should not be unmapped during the lifetime of the uncached version, as then the OS won't know that our process still owns the physical memory.
    bytes = ceilToPage(bytes);
    //first, just allocate enough *virtual* memory for the operation. This is done so that we can do the later mapping to a contiguous range of virtual memory:
    void *mem = mmap(
        NULL,   //let kernel place memory where it wants
        bytes,   //length
        PROT_WRITE | PROT_READ, //ask for read and write permissions to memory
        MAP_SHARED | 
        MAP_ANONYMOUS | //no underlying file; initialize to 0
        MAP_NORESERVE | //don't reserve swap space
        MAP_LOCKED, //lock into *virtual* ram. Physical ram may still change!
        -1, // File descriptor
    0); //no offset into file (file doesn't exist).
    //now, free the virtual memory and immediately remap it to the physical addresses used in virtaddr
    munmap(mem, bytes); //Might not be necessary; MAP_FIXED indicates it can map an already-used page
    for (int offset=0; offset<bytes; offset += PAGE_SIZE) {
        void *mappedPage = mmap((void*)((uintptr_t)mem+offset), PAGE_SIZE, PROT_WRITE|PROT_READ, MAP_SHARED|MAP_FIXED|MAP_NORESERVE|MAP_LOCKED, memfd, virtToUncachedPhys((void*)((uintptr_t)virtaddr+offset), pagemapfd));
        if ((uintptr_t)mappedPage != (uintptr_t)mem+offset) { //We need these mappings to be contiguous over virtual memory (in order to replicate the virtaddr array), so we must ensure that the address we requested from mmap was actually used.
            printf("Failed to create an uncached view of memory at addr %p+0x%08x\n", virtaddr, offset);
            exit(1);
        }
    }
    printf("Created uncached memory view, memsetting it to zero\n");
    usleep(1000*1000);
    printf("Sleep before memset done\n");
    memset(mem, 0, bytes); //Although the cached version might have been reset, those writes might not have made it through.
    printf("Memset done\n");
    usleep(1000*1000);
    printf("Memset done and slept some\n");
    return mem;
}

int main() {
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    int pagemapfd = open("/proc/self/pagemap", O_RDONLY);
    if (memfd < 0 || pagemapfd < 0) {
        printf("Failed to open /dev/mem or /proc/self/pagemap (did you remember to run as root?)\n");
        exit(1);
    }

    printf("Allocating uncached memory\n");
    void *cachedPage = (void*)makeLockedMem(PAGE_SIZE);
    void *uncachedPage = (void*)makeUncachedMemView(cachedPage, PAGE_SIZE, memfd, pagemapfd);
    printf("Done\n");
    exit(0);
}
