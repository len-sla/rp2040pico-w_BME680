#include "cloud_predictor.h"
#include "model.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

CloudPredictor::CloudPredictor() {
    error_buffer[0] = '\0';
}

float CloudPredictor::predict(float temperature, float humidity, float pressure) {
    // Set up logging
    tflite::MicroErrorReporter micro_error_reporter;

    // Map the model into a usable data structure
    const tflite::Model* model = tflite::GetModel(_content_drive_MyDrive_tinyML_output_3_quantized_model_tflite);
    
    // Create resolver for your model operations
    tflite::MicroMutableOpResolver<2> resolver;
    resolver.AddFullyConnected();
    resolver.AddRelu();

    // Allocate memory for input, output, and intermediate tensors
    constexpr int kTensorArenaSize = 32768;
    static uint8_t tensor_arena[kTensorArenaSize];

    // Build an interpreter to run the model
    tflite::MicroInterpreter interpreter(
        model, 
        resolver, 
        tensor_arena, 
        kTensorArenaSize,
        nullptr  // resource_variables
    );

    // Allocate tensors
    if (interpreter.AllocateTensors() != kTfLiteOk) {
        snprintf(error_buffer, sizeof(error_buffer), "AllocateTensors() failed");
        return -1.0f;
    }

    // Get pointers to input and output tensors
    TfLiteTensor* input = interpreter.input(0);
    
    // Fill input tensor with normalized sensor data
    input->data.f[0] = temperature;
    input->data.f[1] = humidity;
    input->data.f[2] = pressure;

    // Run inference
    if (interpreter.Invoke() != kTfLiteOk) {
        snprintf(error_buffer, sizeof(error_buffer), "Invoke() failed");
        return -1.0f;
    }

    // Get output
    TfLiteTensor* output = interpreter.output(0);
    float prediction = output->data.f[0];

    return prediction;
}

const char* CloudPredictor::get_last_error() const {
    return error_buffer;
}
