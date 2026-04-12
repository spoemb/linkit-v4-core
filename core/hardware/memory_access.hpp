#pragma once

/**
 * @file memory_access.hpp
 * @brief Abstract interface for bounds-checked physical memory access.
 */

class MemoryAccess {
public:
	virtual ~MemoryAccess() = default;

	/**
	 * @brief Validate address range and return a physical pointer.
	 * @param addr    Start address.
	 * @param length  Number of bytes to access.
	 * @return Pointer to the physical address.
	 * @throws ILLEGAL_MEMORY_ADDRESS if the range is out of bounds.
	 */
	virtual void *get_physical_address(unsigned int addr, unsigned int length) = 0;
};
