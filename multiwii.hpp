#include "multiwii.h"

struct MspRequest
{
    char preamble[2];
    char direction;

    uint8_t size;
    MspCommand command;
};

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

std::string GetLastErrorAsString()
{
    //Get the error message, if any.
    DWORD errorMessageID = ::GetLastError();
    if (errorMessageID == 0)
        return std::string(); //No error message has been recorded

    LPSTR messageBuffer = nullptr;
    size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)& messageBuffer, 0, NULL);

    std::string message(messageBuffer, size);

    //Free the buffer.
    LocalFree(messageBuffer);

    return message;
}

template<typename T>
T* send_raw_command(ceSerial *serial, MspCommand command, T* params)
{
    uint8_t param_size = 0;
    char* param_data = (char*)params;

    if (params != NULL)
    {
        param_size = sizeof(T);

        if (param_size > std::numeric_limits<uint8_t>::max())
        {
            slog::err << "Invalid parameter size: " << param_size << slog::endl;
            return NULL;
        }
    }

    MspRequest request = {
        .preamble = { '$', 'M' },
        .direction = '<',

        .size = param_size,
        .command = command,
    };

    char* result = (char*)&request;
    size_t result_size = sizeof(request);

    if (params != NULL)
    {
        // request + params
        result = concat_arrays(
            result, result_size,
            param_data, param_size
        );
        result_size += param_size;
    }

    uint8_t crc = calc_checksum(command, param_data, param_size);

    // + crc
    size_t crc_size = sizeof(crc);
    result = concat_arrays(
        result, result_size,
        (char*)&crc, crc_size
    );
    result_size += crc_size;

    // Write to serial port
    size_t write_length = serial->Write(result, result_size);

    slog::info << "Sending: " << slog::endl;
    print_bytes(result, result_size);

    if (write_length != result_size)
    {
        slog::err << "Error writing " << (int)result_size << " only wrote " << (int)write_length << slog::endl;
        slog::err << "Message: " << GetLastErrorAsString();
        print_bytes(result, result_size);
        return NULL;
    }

    serial->Flush();

    // Size of request + params + checksum
    // If we are sending data, only an ack will be sent back (no data)
    size_t rcv_param_size = params == NULL ? sizeof(T) : 0;
    size_t rcv_size = sizeof(MspRequest) + rcv_param_size + sizeof(uint8_t);

    char* rcv_data = new char[rcv_size];
    size_t read_length = serial->Read(rcv_data, rcv_size);

    // Verify response
    // Verify checksum
    // Return params
    // $M> <size> <command> | <params> <checksum>

    MspRequest rcv_request;
    memcpy(&rcv_request, rcv_data, sizeof(MspRequest));

    slog::info << "Rcving: " << slog::endl;
    print_bytes(rcv_data, rcv_size);

    if (read_length != rcv_size)
    {
        slog::err << "Error reading " << (int)rcv_size << " only read " << (int)read_length << " (response told us to read " << (int)rcv_request.size << " bytes)" << slog::endl;
        slog::err << "Message: " << GetLastErrorAsString() << slog::endl;
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

    T* rcv_params = new T;
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

// template<typename T>
// void receive_parameters(int fd, MspCommand command, T* send_params)
// {
//     send_command<T>(fd, command, send_params);

//     // Size of request + checksum
//     size_t rcv_size = sizeof(MspRequest) + sizeof(uint8_t);

//     char* data = new char[rcv_size];
//     unsigned int read_length = read(fd, data, rcv_size);

//     print_bytes(data, rcv_size);

//     if (read_length != rcv_size)
//     {
//         slog::err << "Error reading " << (int)rcv_size << " only read " << (int)read_length << ": " << strerror(errno) << slog::endl;
//         print_bytes(data, rcv_size);
//         return;
//     }

//     // Verify response
//     // Verify checksum
//     // Return params
//     // $M> <size> <command> | <params> <checksum>

//     // printf("Recv: ");

//     // // Print out raw bytes
//     // for (unsigned int i = 0; i < rcv_size; i++)
//     //     printf("%02X ", (unsigned char)data[i]);

//     // printf("\n");

//     MspRequest request;
//     memcpy(&request, data, sizeof(MspRequest));

//     // Verify direction is correct and command matches the one we requested
//     if (request.direction != '>' || request.command != command)
//     {
//         slog::err << "Was expecting a response to our request but received different" << slog::endl;
//         print_bytes(data, rcv_size);
//         return;
//     }

//     if (request.size != 0)
//     {
//         slog::err << "Was expecting a response size of 0 but received " << (int)request.size << " instead" << slog::endl;
//         print_bytes(data, rcv_size);
//         return;
//     }

//     uint8_t calc_crc = calc_checksum(request.command, new char[0], request.size);
//     uint8_t actual_crc;

//     memcpy(&actual_crc, data + sizeof(MspRequest), sizeof(actual_crc));

//     // Verify the checksum is correct
//     if (calc_crc != actual_crc)
//     {
//         slog::err << "Bad checksum: calculated " << (int)calc_crc << ", recv " << (int)actual_crc << slog::endl;
//         return;
//     }
// }

// template<typename T>
// T* receive_parameters(int fd, MspCommand command, T* send_params)
// {
//     send_command<T>(fd, command, send_params);

//     // Size of request + params + checksum
//     // If we are sending data, only an ack will be sent back (no data)
//     size_t param_size = send_params == NULL ? sizeof(T) : 0;
//     size_t rcv_size = sizeof(MspRequest) + param_size + sizeof(uint8_t);

//     char* data = new char[rcv_size];
//     unsigned int read_length = read(fd, data, rcv_size);

//     // Verify response
//     // Verify checksum
//     // Return params
//     // $M> <size> <command> | <params> <checksum>

//     MspRequest request;
//     memcpy(&request, data, sizeof(MspRequest));

//     if (read_length != rcv_size)
//     {
//         slog::err << "Error reading " << (int)rcv_size << " only read " << (int)read_length << " (response told us to read " << (int)request.size << " bytes)" << slog::endl;
//         slog::err << "Message: " << strerror(errno) << slog::endl;
//         print_bytes(data, rcv_size);
//         return NULL;
//     }

//     // Verify direction is correct and command matches the one we requested
//     if (request.direction != '>' || request.command != command)
//     {
//         slog::err << "Was expecting a response to our request but received different" << slog::endl;
//         print_bytes(data, rcv_size);
//         return NULL;
//     }

//     if (request.size != param_size)
//     {
//         slog::err << "Was expecting a response size of " << (int)param_size << " but received " << (int)request.size << " instead" << slog::endl;
//         print_bytes(data, rcv_size);
//         return NULL;
//     }

//     T* params = new T;
//     memcpy(params, data + sizeof(MspRequest), param_size);

//     uint8_t calc_crc = calc_checksum(request.command, (char*)params, request.size);
//     uint8_t actual_crc;

//     memcpy(&actual_crc, data + sizeof(MspRequest) + param_size, sizeof(actual_crc));

//     // Verify the checksum is correct
//     if (calc_crc != actual_crc)
//     {
//         slog::err << "Bad checksum: calculated " << (int)calc_crc << ", recv " << (int)actual_crc << slog::endl;
//         return NULL;
//     }

//     return params;
// }

template<typename T>
void send_command(ceSerial* serial, MspCommand command, T* params)
{
    send_raw_command<T>(serial, command, params);
}

template<typename T>
T* receive_parameters(ceSerial* serial, MspCommand command)
{
    return send_raw_command<T>(serial, command, NULL);
}
