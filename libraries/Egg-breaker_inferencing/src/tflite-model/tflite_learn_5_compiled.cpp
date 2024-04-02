/* Generated by Edge Impulse
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// Generated on: 01.04.2024 17:09:04

#include <stdio.h>
#include <stdlib.h>
#include "edge-impulse-sdk/tensorflow/lite/c/builtin_op_data.h"
#include "edge-impulse-sdk/tensorflow/lite/c/common.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#if EI_CLASSIFIER_PRINT_STATE
#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C" {
    extern void ei_printf(const char *format, ...);
}
#else
extern void ei_printf(const char *format, ...);
#endif
#endif

#if defined __GNUC__
#define ALIGN(X) __attribute__((aligned(X)))
#elif defined _MSC_VER
#define ALIGN(X) __declspec(align(X))
#elif defined __TASKING__
#define ALIGN(X) __align(X)
#elif defined __ICCARM__
#define ALIGN(x) __attribute__((aligned(x)))
#endif

#ifndef EI_MAX_SCRATCH_BUFFER_COUNT
#ifndef CONFIG_IDF_TARGET_ESP32S3
#define EI_MAX_SCRATCH_BUFFER_COUNT 4
#else
#define EI_MAX_SCRATCH_BUFFER_COUNT 4
#endif // CONFIG_IDF_TARGET_ESP32S3
#endif // EI_MAX_SCRATCH_BUFFER_COUNT

#ifndef EI_MAX_OVERFLOW_BUFFER_COUNT
#define EI_MAX_OVERFLOW_BUFFER_COUNT 10
#endif // EI_MAX_OVERFLOW_BUFFER_COUNT

using namespace tflite;
using namespace tflite::ops;
using namespace tflite::ops::micro;

namespace {

#if defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX) || defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX_GNU)
constexpr int kTensorArenaSize = 1456;
#else
constexpr int kTensorArenaSize = 432;
#endif

#if defined(EI_CLASSIFIER_ALLOCATION_STATIC)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX)
#pragma Bss(".tensor_arena")
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#pragma Bss()
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX_GNU)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16) __attribute__((section(".tensor_arena")));
#else
#define EI_CLASSIFIER_ALLOCATION_HEAP 1
uint8_t* tensor_arena = NULL;
#endif

static uint8_t* tensor_boundary;
static uint8_t* current_location;

template <int SZ, class T> struct TfArray {
  int sz; T elem[SZ];
};

enum used_operators_e {
  OP_FULLY_CONNECTED, OP_SOFTMAX,  OP_LAST
};

struct TensorInfo_t { // subset of TfLiteTensor used for initialization from constant memory
  TfLiteAllocationType allocation_type;
  TfLiteType type;
  void* data;
  TfLiteIntArray* dims;
  size_t bytes;
  TfLiteQuantization quantization;
};

typedef struct {
  TfLiteTensor tensor;
  int16_t index;
} TfLiteTensorWithIndex;

typedef struct {
  TfLiteEvalTensor tensor;
  int16_t index;
} TfLiteEvalTensorWithIndex;

TfLiteContext ctx{};
static const int MAX_TFL_TENSOR_COUNT = 4;
static TfLiteTensorWithIndex tflTensors[MAX_TFL_TENSOR_COUNT];
static const int MAX_TFL_EVAL_COUNT = 4;
static TfLiteEvalTensorWithIndex tflEvalTensors[MAX_TFL_EVAL_COUNT];
TfLiteRegistration registrations[OP_LAST];

namespace g0 {
const TfArray<2, int> tensor_dimension0 = { 2, { 1,78 } };
const TfArray<1, float> quant0_scale = { 1, { 0.60858839750289917, } };
const TfArray<1, int> quant0_zero = { 1, { -123 } };
const TfLiteAffineQuantization quant0 = { (TfLiteFloatArray*)&quant0_scale, (TfLiteIntArray*)&quant0_zero, 0 };
const ALIGN(8) int32_t tensor_data1[2] = { -9, 9, };
const TfArray<1, int> tensor_dimension1 = { 1, { 2 } };
const TfArray<1, float> quant1_scale = { 1, { 0.00071094534359872341, } };
const TfArray<1, int> quant1_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant1 = { (TfLiteFloatArray*)&quant1_scale, (TfLiteIntArray*)&quant1_zero, 0 };
const ALIGN(16) int8_t tensor_data2[2*10] = { 
  75, -107, 75, 109, 127, -83, -46, -111, 9, 92, 
  73, 16, -23, -22, -99, 0, -122, 57, -97, 38, 
};
const TfArray<2, int> tensor_dimension2 = { 2, { 2,10 } };
const TfArray<1, float> quant2_scale = { 1, { 0.0053835436701774597, } };
const TfArray<1, int> quant2_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant2 = { (TfLiteFloatArray*)&quant2_scale, (TfLiteIntArray*)&quant2_zero, 0 };
const ALIGN(16) int32_t tensor_data3[10] = { -15, 12, 0, -12, -12, 0, 0, 0, -18, 0, };
const TfArray<1, int> tensor_dimension3 = { 1, { 10 } };
const TfArray<1, float> quant3_scale = { 1, { 0.00054091040510684252, } };
const TfArray<1, int> quant3_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant3 = { (TfLiteFloatArray*)&quant3_scale, (TfLiteIntArray*)&quant3_zero, 0 };
const ALIGN(16) int8_t tensor_data4[10*35] = { 
  41, -84, -77, 13, 108, 15, 120, -114, -6, -58, 22, 24, 66, 121, 56, -90, 0, -29, -77, 27, 96, -105, 70, 28, -122, 32, 61, 101, -65, 58, 77, 89, -105, -88, 113, 
  -59, -124, 77, 73, 58, -45, -53, -105, -101, 23, -54, -63, 71, 26, 79, 98, -55, 37, -48, -33, 6, -65, -105, 59, 123, 47, -74, -73, -73, -50, 116, -25, -64, -104, 19, 
  -25, -105, -3, 115, -53, -64, -39, 22, -45, -12, 35, 55, -23, -94, -14, -76, 92, -106, -109, -88, -55, 74, -7, -127, 122, -42, -43, 62, -8, 84, 16, 68, -38, 55, 24, 
  62, 15, 99, 4, -72, 82, -67, -50, -31, 77, 106, 56, -82, -105, -7, -49, -29, -59, 115, 92, 119, 95, -77, -36, 33, -98, 15, 19, -15, 62, 76, 113, 58, 86, 39, 
  -52, 65, -108, -47, -92, 74, -43, 29, -100, 64, 96, 66, -24, 51, 24, -103, -53, 109, 79, 63, -111, -9, 94, -87, -29, -88, -28, 56, 64, -79, 39, 66, -104, 19, 61, 
  -96, 76, -76, 55, -33, -127, -7, 85, -127, -51, 84, 65, -55, 68, -26, -74, -40, 67, -74, 4, -124, 94, 70, -8, -35, 44, -9, 4, 20, -78, -120, 81, 6, -121, 56, 
  100, 51, -64, -85, 87, 86, 12, -36, -29, 3, 68, 44, -78, -45, 10, -19, 65, -19, -18, -38, -12, -34, 0, -106, 73, 37, -66, 83, -123, 89, -72, 29, -21, -59, 57, 
  -16, 99, 51, 67, -13, 94, 95, -121, 124, -33, -64, 123, 72, -29, -109, 54, 94, 6, 15, 91, 12, 64, -69, -66, 72, 105, -35, 125, -98, -52, -45, -6, 112, -81, -113, 
  65, 44, 56, 42, 95, 35, -122, -28, -32, 10, 100, 26, -9, -118, -41, 104, -95, 77, -52, -36, 21, -107, 31, 68, 115, -98, 64, -119, 25, -52, -61, 126, 19, 7, -122, 
  48, -78, -17, -2, 87, 110, -22, 12, 84, -101, -65, 53, -81, -59, 61, -20, -20, 97, 3, 84, 122, -124, -106, 11, 45, -84, -56, 3, 37, -98, 55, -83, -34, -31, -100, 
};
const TfArray<2, int> tensor_dimension4 = { 2, { 10,35 } };
const TfArray<1, float> quant4_scale = { 1, { 0.0028701003175228834, } };
const TfArray<1, int> quant4_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant4 = { (TfLiteFloatArray*)&quant4_scale, (TfLiteIntArray*)&quant4_zero, 0 };
const ALIGN(16) int32_t tensor_data5[35] = { -8, 0, 5, 4, 0, -5, 8, 0, 0, -6, -6, 9, 6, 8, 7, 0, 0, 0, 0, 0, 5, 0, -4, 0, 1, 6, -7, 0, -6, -6, 10, 7, 0, -6, -2, };
const TfArray<1, int> tensor_dimension5 = { 1, { 35 } };
const TfArray<1, float> quant5_scale = { 1, { 0.0011556693352758884, } };
const TfArray<1, int> quant5_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant5 = { (TfLiteFloatArray*)&quant5_scale, (TfLiteIntArray*)&quant5_zero, 0 };
const ALIGN(16) int8_t tensor_data6[35*78] = { 
  -6, 103, -121, -100, -69, 39, -96, -124, -31, 88, -6, 5, -36, -102, 104, 28, -87, -58, -100, -78, 47, -4, -98, 78, -93, -90, 78, -79, -88, 45, -28, 48, 108, 48, -24, -119, -90, 14, -113, 99, 24, 93, -54, -82, 51, 97, -55, -64, -29, 88, 27, -26, -33, 20, -22, -19, -95, 50, 69, -74, -54, 14, 87, -17, -1, 17, 11, -97, 33, -68, -77, 0, 70, -61, -113, 35, 28, -85, 
  -117, -77, -36, -117, 36, 44, 103, 1, 73, 44, -42, 93, -12, 45, 37, 120, -29, 87, -25, -15, -38, 36, 31, -13, 116, 88, -38, -65, 0, 9, 37, 39, -113, -25, -97, 12, -20, -85, 78, -89, 108, -25, -72, 72, -68, -72, 54, 6, 55, 80, 56, -51, 37, 112, 22, -56, 20, -56, -8, -34, 56, 57, -1, -6, 118, -112, -78, -47, -84, 59, 8, -45, -3, -106, 89, -113, -36, -83, 
  82, 60, -34, 37, 92, 45, -24, -38, -65, -35, -35, 117, -106, -88, 55, -30, -59, -79, 31, -103, -37, 116, -100, -20, -110, -35, -44, -101, -31, -53, -119, -35, 16, -101, 57, -35, 25, -20, 85, -53, -3, -38, 81, 26, 68, 91, 22, 73, 13, 115, -67, -40, 104, -109, -82, -38, 44, -107, 73, -103, -44, -93, 63, -66, 12, 7, 24, -19, 54, -72, 45, 48, 35, 105, 105, -5, 62, 12, 
  -94, -32, -1, 77, 116, 0, 27, 75, 57, -57, -13, 31, 55, -65, -56, 92, 102, -91, -115, 0, 17, 35, 98, -60, 2, 63, -55, -14, 55, 9, -87, 64, -99, -72, -17, -22, 91, 65, 103, 76, 72, 72, -20, -102, 123, -33, -47, -91, -20, 23, 105, 68, 44, 78, -114, 112, 95, -71, 107, 56, -4, -59, -31, -51, -1, -61, 68, -72, -4, 107, -52, 66, 1, -47, 104, 97, 107, -15, 
  98, 91, 23, 71, -29, 87, 116, -121, -83, -49, 16, -77, 110, 95, -113, 6, -110, 95, -66, -4, -59, -110, 96, 27, -105, 67, 73, 49, 81, 31, 39, -72, 76, -101, -69, -66, 110, -91, 103, -83, -104, 80, -49, -91, -33, -105, 63, -102, 39, 43, 9, 22, 4, -82, -103, -3, -89, -72, 110, -43, -63, -114, -48, 57, -71, -75, 73, 66, -15, 119, -13, -66, 112, 0, -80, -70, -22, 14, 
  -39, -25, -114, -51, 122, -99, -11, 30, 22, 32, 56, 29, -97, 17, 28, -56, -60, 28, -46, -30, -115, 82, 27, 42, -86, -107, -78, -111, 75, -115, 81, 82, 37, 115, 99, 40, -94, -83, 20, 50, 111, -72, 100, 105, -22, 94, -88, 32, -14, -125, 58, -65, 4, 98, 112, -70, -6, -12, 100, 46, 16, 1, 115, -33, 32, -65, -76, 100, -78, 0, -19, 95, -23, 91, -94, -54, -46, -68, 
  64, 96, -4, -103, 77, 115, -100, 82, 53, 98, -55, 9, 28, -19, 45, 81, 102, -28, 4, -45, 74, -7, -50, 57, -5, 73, -91, 53, -22, -9, -18, 115, 15, 105, -79, 70, -67, 7, 7, -20, 25, 77, 108, 39, 99, 112, 31, -96, -64, -20, -69, 82, 80, 33, 23, 58, -102, 31, -11, 89, -14, -71, -16, -20, 84, -29, 46, 13, -37, -108, -68, -4, -6, -94, 112, 76, 62, 78, 
  88, -103, 85, -104, 48, -10, -43, 30, -79, 39, 31, -33, -77, -94, -65, 8, -114, -61, -91, 108, -112, -8, 84, -15, 121, 43, -79, 68, 58, -27, 111, -79, -104, 14, -57, -87, -87, 70, -13, 10, -73, 9, 110, -119, 68, -72, -97, 102, 30, -30, 28, 73, -103, -42, 86, 43, 5, -115, 49, 38, -42, 85, 32, -7, 20, -74, 36, -40, 46, 97, 31, 39, -81, -100, 105, 64, 19, 58, 
  66, -63, -118, 117, -71, -101, -111, 106, 19, -91, -71, -45, 81, -45, -52, -81, 99, -53, -52, -116, -34, -115, -93, 111, -109, 114, -19, 68, 36, -4, 54, 109, 16, -49, 86, -41, -48, -108, 105, 23, -51, -65, -55, -88, -54, 116, 34, -36, 101, -10, 12, -118, -109, 15, -87, -53, 53, -39, 119, -89, 86, -32, 6, 76, 21, 7, -90, -69, -31, 96, -32, 108, 22, 29, 9, -94, -34, -59, 
  -106, 19, -64, -75, 8, 66, -102, -108, -82, 64, -106, -101, 70, -21, -3, -22, -7, 64, 15, 71, 34, -113, 5, -123, -68, 64, -102, -78, -94, 83, -104, -95, 63, 100, 96, 45, 86, 62, -25, -25, -80, 77, -21, 52, -38, -51, 89, -85, -89, 22, -16, 20, 42, 75, 33, 89, -95, -30, 88, 84, -9, -33, -12, -58, -55, 40, 10, -106, 56, -35, 48, -88, 82, -59, -3, -76, -38, -32, 
  111, 21, 29, 29, 28, 22, 38, -72, 52, -114, -63, 65, 50, -62, 81, -58, 82, -14, 68, 60, 103, -73, -80, 66, 3, 18, -47, -31, 46, -21, -29, 44, -83, 63, 40, -80, 76, -6, 33, 7, 89, 76, -30, 81, -80, -3, -8, 12, -56, 31, -94, 116, 108, 89, -116, -103, -110, -20, 20, -41, -60, -84, -61, 32, 36, 80, -60, -108, -73, 65, 106, 41, -92, -62, -107, -40, -117, 101, 
  -11, -49, -28, -24, -42, 15, 55, 34, -38, -42, 127, -41, -29, 83, 52, -15, 126, 86, 108, -67, -22, 55, -49, 5, 109, -54, 94, -116, -6, 104, -40, -12, 66, 71, -16, 110, -27, 38, -113, 2, -87, -38, 22, 69, 102, -48, -115, -116, -108, 79, 28, -24, -90, 116, 73, 16, -15, -67, 4, -12, 51, -76, -16, 72, 54, 34, 23, 112, 47, 31, -57, 118, -59, -28, -37, 102, -19, 100, 
  -50, 34, 57, -34, -126, 105, 24, -38, -37, -35, 7, -60, 73, -2, -35, -57, 104, 16, 64, 33, 77, 35, -89, -112, -95, 44, -58, 115, 120, -29, 0, 39, -78, 83, 114, -89, 85, 38, -62, 42, 3, -18, -17, 34, 95, 62, -62, -103, 115, 53, -91, 56, 113, -65, 83, 54, 80, -35, -58, -69, -97, -102, -17, -105, 8, 105, 62, 121, 66, 33, -112, 6, -116, -20, 71, 56, -4, 75, 
  34, 70, -5, 15, 104, -14, 101, -98, -18, 55, 45, 52, -90, 53, 56, 0, -15, 58, 105, -104, 33, 105, 86, -84, 103, -101, 0, 26, 76, -113, -117, 81, 25, -7, 113, -58, -88, -109, -98, 11, -95, 25, 31, 42, -51, 106, 82, 109, 70, -1, 77, -92, 30, -23, -42, 71, 5, -79, 101, 16, 56, 19, 38, -77, -68, 98, -103, 54, -101, 99, 42, 38, -92, -103, -41, -50, 1, 2, 
  114, -102, -90, -36, 32, -117, 27, 88, -113, -8, -5, 38, 122, 115, 0, -74, 98, -63, 110, -72, -77, 94, 72, -76, -113, -48, 92, 32, -50, 118, -28, 60, 112, -105, -98, 86, 114, -23, 124, -83, 16, 110, 53, 46, 34, 87, -111, 1, 26, -79, 84, 5, 41, -11, 6, -76, 100, 79, 32, -88, -39, -51, 42, 125, 120, 107, -41, -64, 103, -115, -11, 39, -4, -56, -34, 6, 0, 59, 
  0, 54, -41, 34, 20, 40, -43, -78, 57, -2, -37, 103, 101, 94, -31, -59, -23, -115, 93, -113, 26, -94, 105, -11, 35, -39, -89, -78, 83, -75, -49, 76, 105, 27, 31, 13, -52, -117, 95, -15, -104, -27, -119, 82, 27, 119, 44, 36, 120, -42, 118, 41, 119, -63, -10, 63, 64, 101, 97, -32, -76, -108, 66, 34, -118, -115, 59, -119, -13, -101, 18, -21, -70, 56, 9, 110, 102, 36, 
  2, 107, -74, 37, -87, -28, -9, 22, -42, -50, 118, -20, -69, 4, -12, 86, -84, -10, 1, 82, -64, 30, 120, -66, 21, -63, 116, 10, -48, 81, 74, -121, -18, 13, -75, -48, 7, 72, 76, -32, -99, -68, 35, -37, 32, 20, 74, -1, -8, -78, -56, 37, 117, -90, -116, -98, 106, -15, 5, -83, 86, 118, -6, -105, 84, -109, 67, -89, -102, 107, 1, -11, -13, 120, 51, 75, 26, 25, 
  -23, 1, 104, 73, 100, 58, 115, 58, -86, 15, -81, 119, -51, 75, 115, 59, -9, -51, 25, -23, 119, -79, -41, 62, 121, -120, 24, 42, 59, -106, 18, -98, 90, -23, -81, -19, 113, 30, -50, -40, 92, 55, -2, -101, -90, -104, -119, -4, 63, -110, 80, -84, 116, 20, -52, 60, 108, -82, -6, 55, -121, 96, -44, 62, 93, -81, 90, -27, -17, -12, -101, 100, 55, -66, 69, 23, 94, 110, 
  -101, -66, -103, -12, -11, 28, 73, 74, -90, -121, 119, -78, 100, -39, 77, 71, -108, 37, -18, 72, 95, -9, 11, 73, -72, 101, -42, 71, -111, 56, -76, -92, -79, 60, 77, 113, 94, 92, 86, -110, 75, 76, -37, -30, 22, -70, -69, -37, 58, 87, 69, 48, 95, 65, -8, -24, 114, 21, -55, -57, 3, -12, -11, 19, -119, -79, 40, 83, -107, -45, -36, 5, -26, 43, -109, -31, -7, -110, 
  -4, -65, -70, 100, -16, 40, -83, -53, -2, -107, 85, -14, -111, 87, 81, 81, 43, 121, -88, -39, 42, 27, 18, 36, 32, -19, 78, -24, 4, 53, -114, 115, 64, 104, -107, 109, 43, 108, -34, -29, -115, 108, -22, -40, -118, 10, 116, -85, -24, 90, 58, -45, 19, 67, 110, -27, 120, -111, -104, 108, 76, -107, 44, -6, -102, -110, 95, -65, -9, -17, 103, 7, -108, 109, -55, 47, 68, 62, 
  -12, -88, -90, 14, 31, 44, 40, 60, 110, 25, 48, -88, 105, 53, -27, -14, -105, 22, -93, 30, -73, -110, 35, 47, -35, 86, 12, -3, -22, -8, -6, 75, 92, -84, 35, 83, -14, 54, 58, 56, -98, 103, 2, -10, -13, 48, -80, 97, 17, 14, -43, 17, -57, 79, 8, 41, -45, 94, 59, 29, -43, 51, 12, -7, 79, 73, -30, 34, 57, 24, -88, 114, 9, -31, 80, -73, 99, -46, 
  -8, -48, -92, -39, 52, -106, 111, 70, -64, -95, 108, -57, 98, -51, 48, -99, -88, 44, 5, -94, -95, -103, 64, 26, 84, -14, 27, -12, -109, -15, 41, -13, 95, 78, 91, 86, -29, 55, -30, -65, -68, -62, 96, 56, -8, 24, 109, 36, 9, 6, -15, -86, 107, 20, 35, -72, -106, -18, 90, 115, 88, 113, -102, 64, 36, -110, -111, -101, 30, -17, -24, -97, 46, 108, -73, -11, 112, 2, 
  -20, 110, 68, 30, 114, -19, -115, 42, 78, -90, 94, 93, 60, 118, -34, 27, -96, -9, -119, -46, -105, -66, -76, 32, -21, 35, 106, -72, 61, -72, 20, -79, 13, -30, -44, 56, -38, 59, 40, 118, -120, 71, 30, -50, -62, -96, 49, -8, 17, -49, 2, -33, 23, -63, -110, -106, 33, 101, 54, -19, 103, 110, 16, -101, -98, -85, -62, -50, -64, 107, -67, 41, 76, 110, -11, -29, 57, -28, 
  63, 64, 107, -14, 59, -42, 32, -44, -27, 8, -98, 110, -77, 95, -121, -11, 78, 56, -90, -99, 117, -81, -37, -62, 93, 3, -6, -63, -78, -103, -58, 57, -118, -33, 1, 102, -79, -63, 95, -61, -63, 91, 57, -87, 43, 41, 53, -55, -97, 102, -6, 0, -99, -39, -87, 29, -29, -121, 30, 13, 14, -87, 51, 79, 87, -67, 66, 100, -111, -67, -40, -111, -72, 7, -99, -121, -23, -65, 
  28, 8, 12, -109, -66, 84, -117, -28, 69, 2, 74, -18, -18, -96, -76, 37, -95, -94, -55, 85, -38, 26, -112, 50, -44, 0, -43, 105, 74, 56, -22, 23, 93, -79, 10, -80, 10, -9, 79, 50, 33, -29, 105, 114, -58, -53, 13, -68, -68, 107, -100, 96, 31, -26, -4, -112, -94, -37, -78, 18, -113, -84, -70, -65, -4, 54, 124, 80, 108, 16, -71, 55, -92, -62, -41, -43, -27, 119, 
  95, 99, -108, 74, -55, 1, 51, -89, 3, 92, 48, -97, -6, -108, -4, -34, 122, 27, -40, 123, 85, -8, 109, -13, -30, 101, -104, 71, 106, 51, 105, 110, -11, 74, 8, 82, -28, 7, -66, 37, -50, -26, 54, 38, 80, 37, -67, -77, -88, -10, 72, 1, -69, -10, 87, 46, 76, -117, 118, 62, 121, -62, -65, 8, 73, 113, -35, -41, -115, 27, -39, 60, -42, -14, -18, -88, 28, 9, 
  -39, 16, 92, 115, -34, -82, -43, 104, -24, 54, 47, -33, -59, 58, -35, 54, 26, 121, 89, 28, -113, 27, 0, 83, 2, -35, 89, -103, -82, 0, -92, 65, 95, -101, -83, -8, 52, 85, -50, -32, 102, -13, 85, 47, 48, -20, 103, -69, -39, -92, 109, 55, -53, 2, -61, 75, -16, 109, -25, 70, -21, -26, -75, 21, -123, 100, -39, 71, -68, -41, -18, -37, 48, 107, -26, 45, 76, -80, 
  -118, -85, 88, -72, -39, -90, 117, 64, -116, 70, 111, 56, -64, 15, -93, -58, -18, 79, 4, 74, -22, 69, -101, 18, 116, -23, 11, -18, -82, -21, 64, -118, 82, -33, 36, -92, 63, 25, 3, 82, 20, 101, 56, -47, -63, 50, -110, 77, -114, -2, 27, -86, 16, 119, 63, 12, 110, -103, 9, -3, -54, 11, -119, -40, 54, -99, 83, 106, -39, 120, 106, 48, -100, -7, 62, 97, -81, -59, 
  110, 71, 53, -105, -1, 93, -51, 27, -57, -63, -71, -98, 102, 71, -100, 99, -108, 59, -80, -20, -108, 20, -43, 56, -49, -95, 117, 120, -111, -27, -72, -51, 52, -18, -43, -25, 2, 63, -103, 111, -41, -91, -118, -70, -71, 20, 47, -45, 22, -50, -10, 5, -113, 95, -90, 109, -69, -14, 25, -77, -34, 18, 63, -113, 51, 104, 49, 66, 80, 3, 18, 17, 113, 34, -110, 28, -88, -83, 
  50, -75, -19, -58, -64, -39, 92, 113, 93, 80, -62, 103, 26, -44, 4, -98, 51, -77, 97, -23, 77, 21, -20, 94, 25, -79, -117, -114, 60, -97, 37, -83, 9, 66, -96, -75, 0, -53, -83, 58, 79, 43, -30, -50, -125, 84, -13, -61, 87, -91, 30, -59, -39, 33, -102, 89, -56, -111, 77, -105, -126, -85, 67, -8, -57, -17, -10, 88, 91, -73, -65, -99, -44, 44, -90, -10, -107, 56, 
  33, -12, 62, -8, -1, -106, -100, 61, -66, 36, -57, -110, -110, 94, -62, -38, 88, -6, -92, -3, -28, 56, 60, -51, -31, -111, 110, -94, -31, 46, -94, -62, -45, 15, -42, 27, 34, -40, 104, 84, 19, -87, -59, -46, 61, -87, -14, 119, 87, -1, -105, 3, 62, 11, 120, 66, 64, 43, 67, 88, 9, 47, 100, -96, 64, 29, 40, 2, -53, -86, -113, -22, 33, -102, -64, 31, 43, 59, 
  45, -67, 19, -36, 83, -1, 20, 63, 53, -65, -76, -106, 110, -44, -27, 87, 50, -111, 119, -32, -80, -93, -46, -16, -15, -99, -79, -13, -86, -100, -12, -111, -18, -39, 86, -67, 85, -110, 64, 118, 6, 51, 102, -54, 56, 93, -58, -36, -104, 34, -25, 18, -114, -37, -51, -33, -31, -65, -13, 110, 16, -61, -31, 33, 41, -23, 20, -65, 123, -95, 59, 27, -87, -97, -31, -14, -102, -93, 
  -115, 102, -63, -47, -97, -26, -87, 85, 89, -64, 51, 64, -75, -27, 115, 39, -32, 111, -86, 11, 97, 82, 61, -35, -59, -12, -96, -91, 80, -23, 63, -90, 36, -119, -41, 58, 19, 28, -36, -34, -57, 48, -103, -97, -104, 119, 43, 21, 91, 52, 0, -2, 24, -99, 38, -43, -28, 70, -85, -69, -8, 112, -57, -56, -59, -50, 121, -60, 68, -13, 105, 51, 66, -21, -73, -73, 37, -24, 
  -126, -21, 65, -82, -8, 4, 54, -90, -91, -61, 70, -66, -12, 73, -81, -38, 6, -104, -56, 116, 51, -82, -101, 22, -89, 33, -81, -96, 39, 116, 26, -89, 79, -49, 36, -84, 74, 98, -81, 103, -27, -102, 37, 103, 53, -63, 79, -7, 15, 40, -52, -118, -86, 88, -73, 24, -64, -77, -74, -64, -3, 27, -104, 10, 32, 107, -1, -108, -7, 5, -104, 112, 89, 87, -112, 0, -103, -90, 
  -106, -11, -94, 35, 65, -105, -78, -5, -51, 66, -33, -51, -66, 122, 91, -52, -34, -24, -87, 10, -97, -17, 12, -83, 78, -120, 69, -14, -22, -117, -33, -63, -31, 34, -99, -92, 13, -117, -122, -72, -76, -3, 54, -104, -5, 59, 58, -103, -97, -65, -44, -113, -21, 13, 23, -86, 121, -94, -37, -23, 72, 47, -79, 89, 62, 78, -88, -54, -63, -57, -104, 77, 112, 46, -23, 95, 80, 104, 
};
const TfArray<2, int> tensor_dimension6 = { 2, { 35,78 } };
const TfArray<1, float> quant6_scale = { 1, { 0.001898934249766171, } };
const TfArray<1, int> quant6_zero = { 1, { 0 } };
const TfLiteAffineQuantization quant6 = { (TfLiteFloatArray*)&quant6_scale, (TfLiteIntArray*)&quant6_zero, 0 };
const TfArray<2, int> tensor_dimension7 = { 2, { 1,35 } };
const TfArray<1, float> quant7_scale = { 1, { 0.18846394121646881, } };
const TfArray<1, int> quant7_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant7 = { (TfLiteFloatArray*)&quant7_scale, (TfLiteIntArray*)&quant7_zero, 0 };
const TfArray<2, int> tensor_dimension8 = { 2, { 1,10 } };
const TfArray<1, float> quant8_scale = { 1, { 0.13205899298191071, } };
const TfArray<1, int> quant8_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant8 = { (TfLiteFloatArray*)&quant8_scale, (TfLiteIntArray*)&quant8_zero, 0 };
const TfArray<2, int> tensor_dimension9 = { 2, { 1,2 } };
const TfArray<1, float> quant9_scale = { 1, { 0.05680437758564949, } };
const TfArray<1, int> quant9_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant9 = { (TfLiteFloatArray*)&quant9_scale, (TfLiteIntArray*)&quant9_zero, 0 };
const TfArray<2, int> tensor_dimension10 = { 2, { 1,2 } };
const TfArray<1, float> quant10_scale = { 1, { 0.00390625, } };
const TfArray<1, int> quant10_zero = { 1, { -128 } };
const TfLiteAffineQuantization quant10 = { (TfLiteFloatArray*)&quant10_scale, (TfLiteIntArray*)&quant10_zero, 0 };
const TfLiteFullyConnectedParams opdata0 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs0 = { 3, { 0,6,5 } };
const TfArray<1, int> outputs0 = { 1, { 7 } };
const TfLiteFullyConnectedParams opdata1 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs1 = { 3, { 7,4,3 } };
const TfArray<1, int> outputs1 = { 1, { 8 } };
const TfLiteFullyConnectedParams opdata2 = { kTfLiteActNone, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs2 = { 3, { 8,2,1 } };
const TfArray<1, int> outputs2 = { 1, { 9 } };
const TfLiteSoftmaxParams opdata3 = { 1 };
const TfArray<1, int> inputs3 = { 1, { 9 } };
const TfArray<1, int> outputs3 = { 1, { 10 } };
};

TensorInfo_t tensorData[] = {
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 0), (TfLiteIntArray*)&g0::tensor_dimension0, 78, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant0))}, },
{ kTfLiteMmapRo, kTfLiteInt32, (int32_t*)g0::tensor_data1, (TfLiteIntArray*)&g0::tensor_dimension1, 8, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant1))}, },
{ kTfLiteMmapRo, kTfLiteInt8, (int32_t*)g0::tensor_data2, (TfLiteIntArray*)&g0::tensor_dimension2, 20, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant2))}, },
{ kTfLiteMmapRo, kTfLiteInt32, (int32_t*)g0::tensor_data3, (TfLiteIntArray*)&g0::tensor_dimension3, 40, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant3))}, },
{ kTfLiteMmapRo, kTfLiteInt8, (int32_t*)g0::tensor_data4, (TfLiteIntArray*)&g0::tensor_dimension4, 350, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant4))}, },
{ kTfLiteMmapRo, kTfLiteInt32, (int32_t*)g0::tensor_data5, (TfLiteIntArray*)&g0::tensor_dimension5, 140, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant5))}, },
{ kTfLiteMmapRo, kTfLiteInt8, (int32_t*)g0::tensor_data6, (TfLiteIntArray*)&g0::tensor_dimension6, 2730, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant6))}, },
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 80), (TfLiteIntArray*)&g0::tensor_dimension7, 35, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant7))}, },
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 0), (TfLiteIntArray*)&g0::tensor_dimension8, 10, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant8))}, },
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 16), (TfLiteIntArray*)&g0::tensor_dimension9, 2, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant9))}, },
{ kTfLiteArenaRw, kTfLiteInt8, (int32_t*)(tensor_arena + 0), (TfLiteIntArray*)&g0::tensor_dimension10, 2, {kTfLiteAffineQuantization, const_cast<void*>(static_cast<const void*>(&g0::quant10))}, },
};

#ifndef TF_LITE_STATIC_MEMORY
TfLiteNode tflNodes[4] = {
{ (TfLiteIntArray*)&g0::inputs0, (TfLiteIntArray*)&g0::outputs0, (TfLiteIntArray*)&g0::inputs0, nullptr, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata0)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs1, (TfLiteIntArray*)&g0::outputs1, (TfLiteIntArray*)&g0::inputs1, nullptr, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata1)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs2, (TfLiteIntArray*)&g0::outputs2, (TfLiteIntArray*)&g0::inputs2, nullptr, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata2)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs3, (TfLiteIntArray*)&g0::outputs3, (TfLiteIntArray*)&g0::inputs3, nullptr, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata3)), nullptr, 0, },
};
#else
TfLiteNode tflNodes[4] = {
{ (TfLiteIntArray*)&g0::inputs0, (TfLiteIntArray*)&g0::outputs0, (TfLiteIntArray*)&g0::inputs0, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata0)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs1, (TfLiteIntArray*)&g0::outputs1, (TfLiteIntArray*)&g0::inputs1, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata1)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs2, (TfLiteIntArray*)&g0::outputs2, (TfLiteIntArray*)&g0::inputs2, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata2)), nullptr, 0, },
{ (TfLiteIntArray*)&g0::inputs3, (TfLiteIntArray*)&g0::outputs3, (TfLiteIntArray*)&g0::inputs3, nullptr, const_cast<void*>(static_cast<const void*>(&g0::opdata3)), nullptr, 0, },
};
#endif

used_operators_e used_ops[] =
{OP_FULLY_CONNECTED, OP_FULLY_CONNECTED, OP_FULLY_CONNECTED, OP_SOFTMAX, };


// Indices into tflTensors and tflNodes for subgraphs
const size_t tflTensors_subgraph_index[] = {0, 11, };
const size_t tflNodes_subgraph_index[] = {0, 4, };

// Input/output tensors
static const int in_tensor_indices[] = {
  0, 
};

static const int out_tensor_indices[] = {
  10, 
};


size_t current_subgraph_index = 0;

static void init_tflite_tensor(size_t i, TfLiteTensor *tensor) {
  tensor->type = tensorData[i].type;
  tensor->is_variable = false;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
  tensor->allocation_type = tensorData[i].allocation_type;
#else
  tensor->allocation_type = (tensor_arena <= tensorData[i].data && tensorData[i].data < tensor_arena + kTensorArenaSize) ? kTfLiteArenaRw : kTfLiteMmapRo;
#endif
  tensor->bytes = tensorData[i].bytes;
  tensor->dims = tensorData[i].dims;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
  if(tensor->allocation_type == kTfLiteArenaRw){
    uint8_t* start = (uint8_t*) ((uintptr_t)tensorData[i].data + (uintptr_t) tensor_arena);

    tensor->data.data =  start;
  }
  else {
      tensor->data.data = tensorData[i].data;
  }
#else
  tensor->data.data = tensorData[i].data;
#endif // EI_CLASSIFIER_ALLOCATION_HEAP
  tensor->quantization = tensorData[i].quantization;
  if (tensor->quantization.type == kTfLiteAffineQuantization) {
    TfLiteAffineQuantization const* quant = ((TfLiteAffineQuantization const*)(tensorData[i].quantization.params));
    tensor->params.scale = quant->scale->data[0];
    tensor->params.zero_point = quant->zero_point->data[0];
  }

}

static void init_tflite_eval_tensor(int i, TfLiteEvalTensor *tensor) {

  tensor->type = tensorData[i].type;

  tensor->dims = tensorData[i].dims;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
  auto allocation_type = tensorData[i].allocation_type;
  if(allocation_type == kTfLiteArenaRw) {
    uint8_t* start = (uint8_t*) ((uintptr_t)tensorData[i].data + (uintptr_t) tensor_arena);

    tensor->data.data =  start;
  }
  else {
    tensor->data.data = tensorData[i].data;
  }
#else
  tensor->data.data = tensorData[i].data;
#endif // EI_CLASSIFIER_ALLOCATION_HEAP
}

static void* overflow_buffers[EI_MAX_OVERFLOW_BUFFER_COUNT];
static size_t overflow_buffers_ix = 0;
static void * AllocatePersistentBufferImpl(struct TfLiteContext* ctx,
                                       size_t bytes) {
  void *ptr;
  uint32_t align_bytes = (bytes % 16) ? 16 - (bytes % 16) : 0;

  if (current_location - (bytes + align_bytes) < tensor_boundary) {
    if (overflow_buffers_ix > EI_MAX_OVERFLOW_BUFFER_COUNT - 1) {
      ei_printf("ERR: Failed to allocate persistent buffer of size %d, does not fit in tensor arena and reached EI_MAX_OVERFLOW_BUFFER_COUNT\n",
        (int)bytes);
      return NULL;
    }

    // OK, this will look super weird, but.... we have CMSIS-NN buffers which
    // we cannot calculate beforehand easily.
    ptr = ei_calloc(bytes, 1);
    if (ptr == NULL) {
      ei_printf("ERR: Failed to allocate persistent buffer of size %d\n", (int)bytes);
      return NULL;
    }
    overflow_buffers[overflow_buffers_ix++] = ptr;
    return ptr;
  }

  current_location -= bytes;

  // align to the left aligned boundary of 16 bytes
  current_location -= 15; // for alignment
  current_location += 16 - ((uintptr_t)(current_location) & 15);

  ptr = current_location;
  memset(ptr, 0, bytes);

  return ptr;
}

typedef struct {
  size_t bytes;
  void *ptr;
} scratch_buffer_t;

static scratch_buffer_t scratch_buffers[EI_MAX_SCRATCH_BUFFER_COUNT];
static size_t scratch_buffers_ix = 0;

static TfLiteStatus RequestScratchBufferInArenaImpl(struct TfLiteContext* ctx, size_t bytes,
                                                int* buffer_idx) {
  if (scratch_buffers_ix > EI_MAX_SCRATCH_BUFFER_COUNT - 1) {
    ei_printf("ERR: Failed to allocate scratch buffer of size %d, reached EI_MAX_SCRATCH_BUFFER_COUNT\n",
      (int)bytes);
    return kTfLiteError;
  }

  scratch_buffer_t b;
  b.bytes = bytes;

  b.ptr = AllocatePersistentBufferImpl(ctx, b.bytes);
  if (!b.ptr) {
    ei_printf("ERR: Failed to allocate scratch buffer of size %d\n",
      (int)bytes);
    return kTfLiteError;
  }

  scratch_buffers[scratch_buffers_ix] = b;
  *buffer_idx = scratch_buffers_ix;

  scratch_buffers_ix++;

  return kTfLiteOk;
}

static void* GetScratchBufferImpl(struct TfLiteContext* ctx, int buffer_idx) {
  if (buffer_idx > (int)scratch_buffers_ix) {
    return NULL;
  }
  return scratch_buffers[buffer_idx].ptr;
}

static const uint16_t TENSOR_IX_UNUSED = 0x7FFF;

static void ResetTensors() {
  for (size_t ix = 0; ix < MAX_TFL_TENSOR_COUNT; ix++) {
    tflTensors[ix].index = TENSOR_IX_UNUSED;
  }
  for (size_t ix = 0; ix < MAX_TFL_EVAL_COUNT; ix++) {
    tflEvalTensors[ix].index = TENSOR_IX_UNUSED;
  }
}

static TfLiteTensor* GetTensorImpl(const struct TfLiteContext* context,
                               int tensor_idx) {

  tensor_idx = tflTensors_subgraph_index[current_subgraph_index] + tensor_idx;

  for (size_t ix = 0; ix < MAX_TFL_TENSOR_COUNT; ix++) {
    // already used? OK!
    if (tflTensors[ix].index == tensor_idx) {
      return &tflTensors[ix].tensor;
    }
    // passed all the ones we've used, so end of the list?
    if (tflTensors[ix].index == TENSOR_IX_UNUSED) {
      // init the tensor
      init_tflite_tensor(tensor_idx, &tflTensors[ix].tensor);
      tflTensors[ix].index = tensor_idx;
      return &tflTensors[ix].tensor;
    }
  }

  ei_printf("ERR: GetTensor called beyond MAX_TFL_TENSOR_COUNT (%d)\n", MAX_TFL_TENSOR_COUNT);
  return nullptr;
}

static TfLiteEvalTensor* GetEvalTensorImpl(const struct TfLiteContext* context,
                                       int tensor_idx) {

  tensor_idx = tflTensors_subgraph_index[current_subgraph_index] + tensor_idx;

  for (size_t ix = 0; ix < MAX_TFL_EVAL_COUNT; ix++) {
    // already used? OK!
    if (tflEvalTensors[ix].index == tensor_idx) {
      return &tflEvalTensors[ix].tensor;
    }
    // passed all the ones we've used, so end of the list?
    if (tflEvalTensors[ix].index == TENSOR_IX_UNUSED) {
      // init the tensor
      init_tflite_eval_tensor(tensor_idx, &tflEvalTensors[ix].tensor);
      tflEvalTensors[ix].index = tensor_idx;
      return &tflEvalTensors[ix].tensor;
    }
  }

  ei_printf("ERR: GetTensor called beyond MAX_TFL_EVAL_COUNT (%d)\n", (int)MAX_TFL_EVAL_COUNT);
  return nullptr;
}

class EonMicroContext : public MicroContext {
 public:
 
  EonMicroContext(): MicroContext(nullptr, nullptr, nullptr) { }

  void* AllocatePersistentBuffer(size_t bytes) {
    return AllocatePersistentBufferImpl(nullptr, bytes);
  }

  TfLiteStatus RequestScratchBufferInArena(size_t bytes,
                                           int* buffer_index) {
  return RequestScratchBufferInArenaImpl(nullptr, bytes, buffer_index);
  }

  void* GetScratchBuffer(int buffer_index) {
    return GetScratchBufferImpl(nullptr, buffer_index);
  }
 
  TfLiteTensor* AllocateTempTfLiteTensor(int tensor_index) {
    return GetTensorImpl(nullptr, tensor_index);
  }

  void DeallocateTempTfLiteTensor(TfLiteTensor* tensor) {
    return;
  }

  bool IsAllTempTfLiteTensorDeallocated() {
    return true;
  }

  TfLiteEvalTensor* GetEvalTensor(int tensor_index) {
    return GetEvalTensorImpl(nullptr, tensor_index);
  }

};


} // namespace

TfLiteStatus tflite_learn_5_init( void*(*alloc_fnc)(size_t,size_t) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  tensor_arena = (uint8_t*) alloc_fnc(16, kTensorArenaSize);
  if (!tensor_arena) {
    ei_printf("ERR: failed to allocate tensor arena\n");
    return kTfLiteError;
  }
#else
  memset(tensor_arena, 0, kTensorArenaSize);
#endif
  tensor_boundary = tensor_arena;
  current_location = tensor_arena + kTensorArenaSize;

  EonMicroContext micro_context_;
  
  // Set microcontext as the context ptr
  ctx.impl_ = static_cast<void*>(&micro_context_);
  // Setup tflitecontext functions
  ctx.AllocatePersistentBuffer = &AllocatePersistentBufferImpl;
  ctx.RequestScratchBufferInArena = &RequestScratchBufferInArenaImpl;
  ctx.GetScratchBuffer = &GetScratchBufferImpl;
  ctx.GetTensor = &GetTensorImpl;
  ctx.GetEvalTensor = &GetEvalTensorImpl;
  ctx.ReportError = &MicroContextReportOpError;

  ctx.tensors_size = 11;
  for (size_t i = 0; i < 11; ++i) {
    TfLiteTensor tensor;
    init_tflite_tensor(i, &tensor);
    if (tensor.allocation_type == kTfLiteArenaRw) {
      auto data_end_ptr = (uint8_t*)tensor.data.data + tensorData[i].bytes;
      if (data_end_ptr > tensor_boundary) {
        tensor_boundary = data_end_ptr;
      }
    }
  }

  if (tensor_boundary > current_location /* end of arena size */) {
    ei_printf("ERR: tensor arena is too small, does not fit model - even without scratch buffers\n");
    return kTfLiteError;
  }

  registrations[OP_FULLY_CONNECTED] = Register_FULLY_CONNECTED();
  registrations[OP_SOFTMAX] = Register_SOFTMAX();

  for (size_t g = 0; g < 1; ++g) {
    current_subgraph_index = g;
    for(size_t i = tflNodes_subgraph_index[g]; i < tflNodes_subgraph_index[g+1]; ++i) {
      if (registrations[used_ops[i]].init) {
        tflNodes[i].user_data = registrations[used_ops[i]].init(&ctx, (const char*)tflNodes[i].builtin_data, 0);
      }
    }
  }
  current_subgraph_index = 0;

  for(size_t g = 0; g < 1; ++g) {
    current_subgraph_index = g;
    for(size_t i = tflNodes_subgraph_index[g]; i < tflNodes_subgraph_index[g+1]; ++i) {
      if (registrations[used_ops[i]].prepare) {
        ResetTensors();
        TfLiteStatus status = registrations[used_ops[i]].prepare(&ctx, &tflNodes[i]);
        if (status != kTfLiteOk) {
          return status;
        }
      }
    }
  }
  current_subgraph_index = 0;

  return kTfLiteOk;
}

TfLiteStatus tflite_learn_5_input(int index, TfLiteTensor *tensor) {
  init_tflite_tensor(in_tensor_indices[index], tensor);
  return kTfLiteOk;
}

TfLiteStatus tflite_learn_5_output(int index, TfLiteTensor *tensor) {
  init_tflite_tensor(out_tensor_indices[index], tensor);
  return kTfLiteOk;
}

TfLiteStatus tflite_learn_5_invoke() {
  for (size_t i = 0; i < 4; ++i) {
    ResetTensors();

    TfLiteStatus status = registrations[used_ops[i]].invoke(&ctx, &tflNodes[i]);

#if EI_CLASSIFIER_PRINT_STATE
    ei_printf("layer %lu\n", i);
    ei_printf("    inputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].inputs->size; ix++) {
      auto d = tensorData[tflNodes[i].inputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");

    ei_printf("    outputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].outputs->size; ix++) {
      auto d = tensorData[tflNodes[i].outputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");
#endif // EI_CLASSIFIER_PRINT_STATE

    if (status != kTfLiteOk) {
      return status;
    }
  }
  return kTfLiteOk;
}

TfLiteStatus tflite_learn_5_reset( void (*free_fnc)(void* ptr) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  free_fnc(tensor_arena);
#endif

  // scratch buffers are allocated within the arena, so just reset the counter so memory can be reused
  scratch_buffers_ix = 0;

  // overflow buffers are on the heap, so free them first
  for (size_t ix = 0; ix < overflow_buffers_ix; ix++) {
    ei_free(overflow_buffers[ix]);
  }
  overflow_buffers_ix = 0;
  return kTfLiteOk;
}
