// Copyright (c) 2020, Tobias Rapp
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Karlsruhe Institute of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "extrema.cuh"

#include <thrust/device_vector.h>
#include <thrust/device_ptr.h>
#include <thrust/extrema.h>

namespace cut
{
namespace device
{

void find_minmax_device(const float *values, size_t num_values, float &min, float &max)
{
    thrust::device_ptr<const float> d_ptr(values);

    auto it = thrust::minmax_element(d_ptr, d_ptr + num_values);
    min = *it.first;
    max = *it.second;
}

void find_minmax_host(const float *values, size_t num_values, float &min, float &max)
{
    thrust::device_vector<float> vec(values, values + num_values);

    auto it = thrust::minmax_element(vec.begin(), vec.end());
    min = *it.first;
    max = *it.second;
}

void find_min_host(const float *values, size_t num_values, float &min)
{
    thrust::device_vector<float> vec(values, values + num_values);
    min = *thrust::min_element(vec.begin(), vec.end());
}

void find_max_host(const float *values, size_t num_values, float &max)
{
    thrust::device_vector<float> vec(values, values + num_values);
    max = *thrust::max_element(vec.begin(), vec.end());
}

} // namespace device
} // namespace cut