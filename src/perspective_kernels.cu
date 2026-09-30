#include <cuda_runtime.h>
#include <cmath>
#include <thrust/device_ptr.h>
#include <thrust/sequence.h>

__global__ void computeMapsKernel(
    const float* x_grid,
    const float* y_grid,
    float* full_map_x,
    float* full_map_y,
    int out_height,
    int out_width,
    int img_height,
    int img_width,
    float cx,
    float cy,
    float tan_horizontal,
    float tan_vertical,
    const float* camera_orientation_matrix,
    const float* back_to_front_rotation,
    const float* back_to_front_translation,
    int x_offset_crop,
    int y_offset_crop)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= out_width || y >= out_height) {
        return;
    }

    int idx = y * out_width + x;

    float xg = x_grid[idx];
    float yg = y_grid[idx];

    float X = (xg / out_width) * 2.0f * tan_horizontal - tan_horizontal;
    float Y = (yg / out_height) * 2.0f * tan_vertical - tan_vertical;
    float Z = 1.0f;

    float Xo = camera_orientation_matrix[0] * X + camera_orientation_matrix[1] * Y + camera_orientation_matrix[2] * Z;
    float Yo = camera_orientation_matrix[3] * X + camera_orientation_matrix[4] * Y + camera_orientation_matrix[5] * Z;
    float Zo = camera_orientation_matrix[6] * X + camera_orientation_matrix[7] * Y + camera_orientation_matrix[8] * Z;

    bool is_back = (Zo < 0.0f);

    if (is_back) {
        float Xt = back_to_front_rotation[0] * Xo + back_to_front_rotation[1] * Yo + back_to_front_rotation[2] * Zo + back_to_front_translation[0];
        float Yt = back_to_front_rotation[3] * Xo + back_to_front_rotation[4] * Yo + back_to_front_rotation[5] * Zo + back_to_front_translation[1];
        float Zt = back_to_front_rotation[6] * Xo + back_to_front_rotation[7] * Yo + back_to_front_rotation[8] * Zo + back_to_front_translation[2];
        Xo = Xt;
        Yo = Yt;
        Zo = Zt;
    }

    float r = sqrtf(Xo * Xo + Yo * Yo);
    if (r < 1e-6f) {
        r = 1e-6f;
    }

    float theta = atan2f(r, fabsf(Zo));
    float r_fisheye = 2.0f * theta / M_PI * (img_width / 2.0f);

    float u = cx + (Xo / r) * r_fisheye;
    float v = cy + (Yo / r) * r_fisheye;

    float rot_u;
    float rot_v;

    if (is_back) {
        rot_u = v;
        rot_v = (img_width - 1.0f) - u;
    } else {
        rot_u = (img_height - 1.0f) - v;
        rot_v = u;
    }

    rot_u += x_offset_crop;
    rot_v += y_offset_crop;

    float final_x = is_back ? rot_u : img_width + rot_u;
    float final_y = rot_v;

    full_map_x[idx] = final_x;
    full_map_y[idx] = final_y;
}

extern "C" void perspective_generate_range_kernel(float* out, int size)
{
    thrust::device_ptr<float> ptr(out);
    thrust::sequence(ptr, ptr + size, 0.0f, 1.0f);
}

extern "C" void perspective_compute_maps_kernel(
    const float* x_grid,
    const float* y_grid,
    float* full_map_x,
    float* full_map_y,
    int out_height,
    int out_width,
    int img_height,
    int img_width,
    float cx,
    float cy,
    float tan_horizontal,
    float tan_vertical,
    const float* camera_orientation_matrix,
    const float* back_to_front_rotation,
    const float* back_to_front_translation,
    int x_offset_crop,
    int y_offset_crop)
{
    dim3 block(16, 16);
    dim3 grid((out_width + block.x - 1) / block.x, (out_height + block.y - 1) / block.y);

    computeMapsKernel<<<grid, block>>>(
        x_grid,
        y_grid,
        full_map_x,
        full_map_y,
        out_height,
        out_width,
        img_height,
        img_width,
        cx,
        cy,
        tan_horizontal,
        tan_vertical,
        camera_orientation_matrix,
        back_to_front_rotation,
        back_to_front_translation,
        x_offset_crop,
        y_offset_crop);
    cudaDeviceSynchronize();
}
