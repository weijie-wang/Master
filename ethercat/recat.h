#ifndef _RECAT_H
#define _RECAT_H

#include "platform.h"


#include <ecrt.h>

#include <assert.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <stdlib.h> // for size_t
#include <stdint.h>

struct PDO_CONFIG
{
    ec_pdo_entry_reg_t* reg;
    ec_pdo_entry_info_t *entries;
    ec_pdo_info_t* info;
};

struct PDO_Data
{
    inline PDO_Data(uint16_t _alias, uint16_t _position,
            uint32_t _vendor_id, uint32_t _product_code,
            uint16_t _index, uint8_t _subindex, int _size)
        :alias(_alias), position(_position),
          vendor_id(_vendor_id), product_code(_product_code),
          index(_index), subindex(_subindex), bit_length(_size)
    {

    };
    //PDO_DIR dir;
    uint16_t alias; /**< Slave alias address. */
    uint16_t position; /**< Slave position. */
    uint32_t vendor_id; /**< Slave vendor ID. */
    uint32_t product_code; /**< Slave product code. */
    uint16_t index; /**< PDO entry index. */
    uint8_t subindex; /**< PDO entry subindex. */

    uint32_t bit_length;
    uint32_t offset;
};
#define __READ_WIRTE__(T, x) \
struct PDO_##x: PDO_Data  \
{                         \
    inline PDO_##x(uint16_t _alias, uint16_t _position,         \
            uint32_t _vendor_id, uint32_t _product_code,        \
            uint16_t _index, uint8_t _subindex)       \
        : PDO_Data(_alias,_position, _vendor_id, _product_code, _index, _subindex, sizeof(T) * 8) {}\
    T data;     \
    inline void read(void *ptr){data = EC_READ_##x( ((uint8_t*)ptr) + offset);}\
    inline void write(void *ptr){EC_WRITE_##x( ((uint8_t*)ptr) + offset, data);}\
}

__READ_WIRTE__(uint32_t, U32);
__READ_WIRTE__( int32_t, S32);
__READ_WIRTE__(uint16_t, U16);
__READ_WIRTE__( int16_t, S16);
__READ_WIRTE__( uint8_t, U8);
__READ_WIRTE__(  int8_t, S8);

EXPORT void print_pdo(PDO_Data* p);
EXPORT PDO_CONFIG* pdo_config(uint16_t index, ... );
EXPORT void pdo_free(PDO_CONFIG* config);

#undef __READ_WIRTE__

#include "timer.h"

class EXPORT RECAT : public Callback
{
public:
    ec_master_t * master;
    ec_master_state_t master_state;
    ec_domain_t * domainOUT;
    ec_domain_t * domainIN;
    uint8_t * domain_out_pd;
    uint8_t * domain_in_pd;
    ec_domain_state_t domain1_state;
    unsigned int frequency;
    unsigned int counter;
    Timer* timer;
    std::vector <ec_slave_config_state_t> slave_states;
    std::vector <ec_slave_config_t *> slave_configs;
public:
    RECAT (std::string const & address);
    int slave (uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code);
    void sync_manager (int index, ec_sync_info_t const (syncs) []);
    int pdo (ec_pdo_entry_reg_t const * regs_in, ec_pdo_entry_reg_t const * regs_out);
    int start (int frequency = 1000);
    void config_dc(uint16_t index, uint16_t assign_activate,
                   uint32_t sync0_cycle_time, int32_t sync0_shift_time,
                   uint32_t sync1_cycle_time, int32_t sync1_shift_time);
    void set_time(uint64_t ns);
    void dc_refresh();
    //void add(int index, int address, int size, void* data);

    void pdo_in(uint16_t index, PDO_Data* pdo, ... );
    void pdo_out(PDO_Data* pdo, ... );
    virtual void seconds () = 0;
    virtual void process (uint8_t * input, uint8_t * output) = 0;
    inline int slave_size()
    {
        return this->slave_configs.size();
    }
    int write(uint16_t slave_position,
              uint16_t index, uint8_t subindex,
              uint8_t *data, size_t data_size);
    int read(uint16_t slave_position,
             uint16_t index, uint8_t subindex,
             uint8_t *data, size_t data_size);
private:
    virtual void run ();
    virtual void check ();
    void check_domain1_state ();
    void check_master_state ();
    void check_slave_config_states ();
};


#endif
