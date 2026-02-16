#pragma once

#include "filesystem.hpp"

#define IS25_BLOCK_COUNT   (4096)
#define IS25_BLOCK_SIZE    (4*1024)
#define IS25_PAGE_SIZE     (256)

class Is25Flash : public FlashInterface {
public:
	Is25Flash() : FlashInterface(IS25_BLOCK_COUNT, IS25_BLOCK_SIZE, IS25_PAGE_SIZE), m_is_init(false), m_power_ref_count(0) {}
	void init();

private:
	bool m_is_init;
	unsigned int m_power_ref_count;

	int read(lfs_block_t block, lfs_off_t off, void * buffer, lfs_size_t size) override;
	int prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) override;
	int erase(lfs_block_t block) override;
	int sync() override;

	int _read(lfs_block_t block, lfs_off_t off, void * buffer, lfs_size_t size);
	int _prog(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
	int _prog_fast(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
	int _erase(lfs_block_t block);
	int _sync();

	void _power_up_hw();
	void _power_down_hw();

public:
	// Fast prog for OTA - no sync, no verification
	int prog_fast(lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);

	// Power management - public for OTA optimization and bulk operations
	// Reference-counted: nested power_up/power_down calls are safe.
	// The QSPI peripheral is only initialized on the first power_up
	// and uninitialized when the last power_down brings the count to 0.
	void power_up() override;
	void power_down() override;
};
