#include <iostream>
#include <string.h>
#include "yesense_decoder_comm.h"
#include "yesense_decoder.h"

// ===========================================================================


// ===========================================================================
namespace yesense{

yesense_decoder::yesense_decoder()
{
    memset(decode_data, 0, DATA_BUF_SIZE);
    std_out_decoder = new yis_std_out_decoder();

    cmd_info.flg        = 0u;
    cmd_info.result     = 0x00u;
    cmd_info.data_class = 0x00u;
}

yesense_decoder::~yesense_decoder()
{

}

int yesense_decoder::data_proc(unsigned char *data, unsigned int len, yis_out_data_t *result)
{   
    int ret = analysis_ok;

    if(len + decode_buf_len > DATA_BUF_SIZE)
    {
        return buf_full;
    }

    memcpy(decode_data + decode_buf_len, data, len);
    decode_buf_len += len;

    // start decoding
    ret = std_out_decoder->data_proc(decode_data, decode_buf_len, result);
    if(no_header == ret)
    {
        if(!cmd_info.flg)
        {
            // clear all data
            clear_buf_data(0u, decode_buf_len - 1u);
        }
    }
    else if(data_len_err == ret)
    {
        // msg_idx_t *msg_idx = std_out_decoder->msg_idx_obt();
        // clear_buf_data(msg_idx->st_idx, decode_buf_len - 1u);
    }
    else if(crc_err == ret || analysis_ok == ret)
    {
        msg_idx_t *msg_idx = std_out_decoder->msg_idx_obt();
        clear_buf_data(msg_idx->st_idx, msg_idx->end_idx);   
    }

    // reserved for command decoder
    // if(cmd_info.flg)
    // {

    // }

    return ret;
}

int yesense_decoder::clear_buf_data(unsigned int st_idx, unsigned int end_idx)
{
    unsigned int cnt = 0u;

    if(st_idx > end_idx)
    {
        return para_err;
    }

    cnt = end_idx - st_idx + 1u;
    if(cnt >= DATA_BUF_SIZE)
    {
        memset(decode_data, 0u, DATA_BUF_SIZE);
        decode_buf_len = 0u;
    }
    else
    {
        memcpy(decode_data + st_idx, decode_data + end_idx + 1u, DATA_BUF_SIZE - 1 - end_idx);
        decode_buf_len -= cnt;
    }

    return analysis_ok;
}

}
