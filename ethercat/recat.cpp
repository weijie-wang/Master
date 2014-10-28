#include "recat.h"

/*
StopWatch::StopWatch()
{
    beginticks.QuadPart=0;
    endticks.QuadPart=0;
    frequency.QuadPart=0;
    QueryPerformanceFrequency(&frequency);
    this->Start();
}
StopWatch::~StopWatch()
{
}
void StopWatch::Start()
{
    //beginticks=GetTickCount();
    QueryPerformanceCounter(&beginticks);

}
void StopWatch::Stop()
{
    QueryPerformanceCounter(&endticks);

}
double  StopWatch::GetCostMillisecond()
{
    unsigned long long cost=(unsigned long long)(endticks.QuadPart-beginticks.QuadPart);
    double millsecond=(double)cost*1000.0/(double)frequency.QuadPart;
    return millsecond;
}
unsigned long long StopWatch::GetFrequency()
{
    return (unsigned long long)frequency.QuadPart;
}*/




#include <vector>

RECAT::RECAT (std::string const & address)
    : master (NULL),
      domainOUT (NULL), domainIN (NULL),
      domain_out_pd (NULL), domain_in_pd (NULL)
{
    //printf("here tested\n");
    //ec_init_module();
    //ec_winpcap_init_entry(address.c_str());

    this->master = ecrt_request_master(0);
    assert(this->master);

    this->domainOUT = ecrt_master_create_domain(this->master);
    assert (this->domainOUT);

    this->domainIN = ecrt_master_create_domain(this->master);
    assert (this->domainIN);
    this->timer = new Timer();

}
int RECAT::slave (uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code)
{

    ec_slave_config_t *sc_ana_in =ecrt_master_slave_config(
                this->master,
                alias, position,
                vendor_id, product_code );

    assert( sc_ana_in);
    this->slave_configs.push_back( sc_ana_in );
    this->slave_states.resize(this->slave_configs.size());
    return this->slave_configs.size() - 1;
}
void RECAT::sync_manager (int index, ec_sync_info_t const (syncs) [])
{
    assert(index >= 0);
    assert(index < this->slave_configs.size());
    int ret = ecrt_slave_config_pdos(this->slave_configs[index], EC_END, syncs);
    assert( ret == 0);
}
/*void RECAT::add(int index, int address, int size, void* data)
{
    static ec_datagram_t datagram;
    ec_datagram_init(&datagram);

    snprintf(datagram.name, EC_DATAGRAM_NAME_SIZE, "send");

    if (ec_datagram_prealloc(&datagram, 2) < 0) {
        ec_datagram_clear(&datagram);
        return;
    }

    ec_datagram_apwr(&datagram, 0xffff, 0x0982, 2);
    EC_WRITE_U16(datagram.data, 60000);

    ec_master_queue_datagram(this->master, &datagram);
}*/

void RECAT::config_dc(uint16_t index, uint16_t assign_activate,
                      uint32_t sync0_cycle_time, int32_t sync0_shift_time,
                      uint32_t sync1_cycle_time, int32_t sync1_shift_time)
{
    assert(index >= 0);
    assert(index < this->slave_configs.size());
    ecrt_slave_config_dc(this->slave_configs[index], assign_activate,
                          sync0_cycle_time, sync0_shift_time,
                          sync1_cycle_time, sync1_shift_time);
}
void RECAT::set_time(uint64_t t)
{
    ecrt_master_application_time(this->master, t);
}
void RECAT::dc_refresh()
{
    static int sync_ref_counter = 0;
    if (sync_ref_counter) {
        sync_ref_counter--;
    } else {
        sync_ref_counter = 1; // sync every cycle
        ecrt_master_sync_reference_clock(this->master);
    }
    ecrt_master_sync_slave_clocks(this->master);
}
int RECAT::pdo (ec_pdo_entry_reg_t const * regs_in, ec_pdo_entry_reg_t const * regs_out)
{
    int ret = ecrt_domain_reg_pdo_entry_list(this->domainOUT, regs_out);
    assert( ret == 0);

    ret = ecrt_domain_reg_pdo_entry_list(this->domainIN, regs_in);
    assert( ret == 0);

    return 0;
}
int RECAT::start (int frequency)
{
    this->frequency = frequency;
    int ret = (ecrt_master_activate(this->master));
    assert( ret == 0);

    this->domain_out_pd = ecrt_domain_data(this->domainOUT);
    assert( this->domain_out_pd );

    this->domain_in_pd = ecrt_domain_data(this->domainIN);
    assert( this->domain_in_pd);

    this->counter = this->frequency;

    timer->set_callback(this);
    timer->set_interval(1000000/this->frequency, -1);
    timer->start();

    return 0;
}
void RECAT::check_domain1_state ()
{
    ec_domain_state_t ds;

    ecrt_domain_state(domainOUT, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}
void RECAT::check_master_state ()
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}
void RECAT::check_slave_config_states ()
{
    ec_slave_config_state_t s;

    for( int i = 0; i < this->slave_configs.size(); i ++ )
    {
        ec_slave_config_t *sc_ana_in = this->slave_configs[i];
        ecrt_slave_config_state(sc_ana_in, &s);

        if (s.al_state != this->slave_states[i].al_state)
            printf("AnaIn: State 0x%02X.\n", s.al_state);
        if (s.online != this->slave_states[i].online)
            printf("AnaIn: %s.\n", s.online ? "online" : "offline");
        if (s.operational != this->slave_states[i].operational)
            printf("AnaIn: %soperational.\n",
                   s.operational ? "" : "Not ");

        this->slave_states[i] = s;
    }
}
void RECAT::check ()
{
    // check for master state (optional)
    check_master_state();

    // check for islave configuration state(s) (optional)
    check_slave_config_states();
}
void RECAT::run ()
{
    // receive process data
    ecrt_master_receive(this->master);
    ecrt_domain_process(this->domainOUT);

    ecrt_domain_process(this->domainIN);

    //check_domain1_state();


    if (this->counter > 0)
        this->counter--;
    else
    {
        this->counter = this->frequency;
        //this->check();
        this->seconds();
    }

    ecrt_domain_queue(domainIN);
    this->process(domain_in_pd, domain_out_pd);

    // send process data
    
    ecrt_domain_queue(domainOUT);
    ecrt_master_send(master);
}
int RECAT::write(uint16_t slave_index, uint16_t index, uint8_t subindex, uint8_t *data, size_t data_size)
{
    uint32_t abort_code = 0;
    size_t reslut_size = 0;
    int ret = 0;
    //printf("%x %x %x\n", slave_index, index, subindex);
    ret = ecrt_master_sdo_download(this->master,
                                   slave_index, index, subindex,
                                   (uint8_t*)data,data_size,&abort_code);
    if(ret)
    {
        //printf("write failed: code = %ld\n",abort_code);
        return -1;
    }
    return data_size;
}

int RECAT::read(uint16_t slave_index, uint16_t index, uint8_t subindex, uint8_t *data, size_t data_size)
{
    uint32_t abort_code = 0;
    size_t reslut_size = 0;
    int ret = 0;

    ret = ecrt_master_sdo_upload(this->master,
                                 slave_index, index, subindex,
                                 (uint8_t*)data,data_size,&reslut_size, &abort_code);

    if(ret)
    {
        printf("read failed: code = %ld\n",abort_code);
        return -1;
    }
    return reslut_size;
}

PDO_CONFIG* pdo_config(uint16_t index, ... )
{
    va_list argp;
    int argno = 0;
    ec_pdo_entry_reg_t* reg = (ec_pdo_entry_reg_t*)malloc(sizeof(ec_pdo_entry_reg_t) );
    ec_pdo_entry_info_t *entries = (ec_pdo_entry_info_t *)malloc(argno * sizeof(ec_pdo_entry_info_t));
    memset(reg, 0, sizeof(ec_pdo_entry_reg_t));

    va_start( argp, index );

    while (1)
    {
        PDO_Data* para = va_arg( argp, PDO_Data *);
        if (  para == 0 )
            break;
        argno++;
        reg = (ec_pdo_entry_reg_t*)realloc(reg, sizeof(ec_pdo_entry_reg_t) * (argno + 1) );
        entries = (ec_pdo_entry_info_t*)realloc(entries, sizeof(ec_pdo_entry_info_t) * (argno) );

        reg[argno - 1].alias = para->alias;
        reg[argno - 1].position = para->position;

        reg[argno - 1].vendor_id = para->vendor_id;
        reg[argno - 1].product_code = para->product_code;

        reg[argno - 1].index = para->index;
        reg[argno - 1].subindex = para->subindex;
        reg[argno - 1].offset = &(para->offset);

        entries[argno - 1].index = para->index;
        entries[argno - 1].subindex = para->subindex;
        entries[argno - 1].bit_length = para->bit_length;

        memset(&(reg[argno]), 0, sizeof(ec_pdo_entry_reg_t));
    }
    va_end( argp );
    ec_pdo_info_t* info = (ec_pdo_info_t*) malloc(sizeof(ec_pdo_info_t));
    info->index = index;
    info->n_entries = argno;
    info->entries = entries;

    PDO_CONFIG* config = (PDO_CONFIG*)malloc(sizeof(PDO_CONFIG));

    config->entries = entries;
    config->reg = reg;
    config->info = info;
#ifdef _DEBUG_
    for(int i = 0; i < argno; i++)
    {
        ec_pdo_entry_reg_t* p = config->reg + i;
        printf("alias=%X position=%X vendor_id=%X product_code=%X index=%X subindex=%X offset=%X\n",
               p->alias,p->position, p->vendor_id,   p->product_code, p->index, p->subindex, p->offset);
    }

    for(int i = 0; i < argno; i++)
    {
        ec_pdo_entry_info_t* p = config->entries + i;
        printf("index=%X subindex=%X bit_length=%X\n",
               p->index, p->subindex, p->bit_length );
    }

    printf("index=%X n_entries=%x\n",
           config->info->index, config->info->n_entries);
#endif
    return config;
}
void pdo_free(PDO_CONFIG* config)
{
    if(config->entries)
        free( config->entries );
    if(config->reg)
        free( config->reg );
    if(config->info)
        free( config->info );
}
void print_pdo(PDO_Data* p)
{
    printf("alias=%X position=%X vendor_id=%X product_code=%X index=%X subindex=%X offset=%X bit=%X\n",
           p->alias,p->position, p->vendor_id,   p->product_code, p->index, p->subindex, p->offset, p->bit_length);
}
