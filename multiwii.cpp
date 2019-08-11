#include "multiwii.h"

namespace Msp
{

char* concat_arrays(char* array1, size_t size1, char* array2, size_t size2)
{
    char* result = (char*)malloc(size1 + size2);
    std::copy(array1, array1 + size1, result);
    std::copy(array2, array2 + size2, result + size1);

    return result;
}

uint8_t calc_checksum(MspCommand command, char* params, size_t param_size)
{
    uint8_t crc = param_size ^ command;

    for (unsigned int i = 0; i < param_size; i++)
    {
        crc ^= params[i];
    }

    return crc;
}

void print_bytes(char* data, size_t length)
{
    slog::info << "Raw data: ";

    // Print out raw bytes
    for (unsigned int i = 0; i < length; i++)
        printf("%02X ", (unsigned char)data[i]);

    printf("\n");
}

char* send_raw_command(ceSerial* serial, MspCommand command, char* param_data, uint8_t param_size)
{
    uint8_t send_param_size = param_data != NULL ? param_size : 0;

    MspRequest request = {
        .preamble = { '$', 'M' },
        .direction = '<',

        .size = send_param_size,
        .command = command,
    };

    char* result = (char*)& request;
    size_t result_size = sizeof(request);

    if (param_data != NULL)
    {
        // request + params
        result = concat_arrays(
            result, result_size,
            param_data, send_param_size
        );
        result_size += send_param_size;
    }

    uint8_t crc = calc_checksum(command, param_data, send_param_size);

    // + crc
    size_t crc_size = sizeof(crc);
    result = concat_arrays(
        result, result_size,
        (char*)& crc, crc_size
    );
    result_size += crc_size;

    // Write to serial port
    size_t write_length = serial->Write(result, result_size);

    //slog::info << "Sending: " << slog::endl;
    //print_bytes(result, result_size);

    if (write_length != result_size)
    {
        slog::err << "Error writing " << (int)result_size << " only wrote " << (int)write_length << slog::endl;
        print_bytes(result, result_size);
        return NULL;
    }

    serial->Flush();

    // Size of request + params + checksum
    // If we are sending data, only an ack will be sent back (no data)
    size_t rcv_param_size = param_data == NULL ? param_size : 0;
    size_t rcv_size = sizeof(MspRequest) + rcv_param_size + sizeof(uint8_t);

    char* rcv_data = new char[rcv_size];
    size_t read_length = serial->Read(rcv_data, rcv_size);

    // Verify response
    // Verify checksum
    // Return params
    // $M> <size> <command> | <params> <checksum>

    MspRequest rcv_request;
    memcpy(&rcv_request, rcv_data, sizeof(MspRequest));

    //slog::info << "Rcving: " << slog::endl;
    //print_bytes(rcv_data, rcv_size);

    if (read_length != rcv_size)
    {
        slog::err << "Error reading " << (int)rcv_size << " only read " << (int)read_length << " (response told us to read " << (int)rcv_request.size << " bytes)" << slog::endl;
        print_bytes(rcv_data, rcv_size);
        return NULL;
    }

    // Verify direction is correct and command matches the one we requested
    if (rcv_request.direction != '>' || rcv_request.command != command)
    {
        slog::err << "Was expecting a response to our request but received different" << slog::endl;
        print_bytes(rcv_data, rcv_size);
        return NULL;
    }

    if (rcv_request.size != rcv_param_size)
    {
        slog::err << "Was expecting a response size of " << (int)rcv_param_size << " but received " << (int)rcv_request.size << " instead" << slog::endl;
        print_bytes(rcv_data, rcv_size);
        return NULL;
    }

    char* rcv_params = new char[rcv_param_size];
    memcpy(rcv_params, rcv_data + sizeof(MspRequest), rcv_param_size);

    uint8_t calc_crc = calc_checksum(rcv_request.command, (char*)rcv_params, rcv_request.size);
    uint8_t actual_crc;

    memcpy(&actual_crc, rcv_data + sizeof(MspRequest) + rcv_param_size, sizeof(actual_crc));

    // Verify the checksum is correct
    if (calc_crc != actual_crc)
    {
        slog::err << "Bad checksum: calculated " << (int)calc_crc << ", recv " << (int)actual_crc << slog::endl;
        return NULL;
    }

    return rcv_params;
}

}
