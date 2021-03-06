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
#include "sort_util.cuh"

#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/sequence.h>

using std::vector;

namespace cut
{

namespace device
{

template <typename Value, typename Id>
void getSortedIndicesHelper(const vector<Value> &values, vector<Id> &valueIndices)
{
    valueIndices.resize(values.size());

    thrust::device_vector<Value> dValues(values);

    thrust::device_vector<Id> dIndices(values.size());
    thrust::sequence(dIndices.begin(), dIndices.end());

    thrust::sort_by_key(dValues.begin(), dValues.end(), dIndices.begin());

    CHECK_CUDA(cudaMemcpy(valueIndices.data(), thrust::raw_pointer_cast(dIndices.data()), values.size() * sizeof(Id),
                          cudaMemcpyDeviceToHost));
}

void getSortedIndices(const vector<float> &values, vector<int> &valueIndices)
{
    getSortedIndicesHelper(values, valueIndices);
}

void getSortedIndices(const vector<float> &values, vector<uint> &valueIndices)
{
    getSortedIndicesHelper(values, valueIndices);
}

void getSortedIndices(const vector<uint32_t> &values, vector<uint32_t> &valueIndices)
{
    getSortedIndicesHelper(values, valueIndices);
}

template <typename T, typename I>
__global__ void reorderingKernel(const T *in, T *out, const I *indices, size_t num_values)
{
    auto gId = blockDim.x * blockIdx.x + threadIdx.x;
    if (gId >= num_values)
        return;
    out[gId] = in[indices[gId]];
}

template <typename T, typename I>
inline void reorderByIndexHelper(T *values, const I *indices, size_t num_values)
{
    dim3 threadsPerBlock = 256;
    dim3 numPartBlocks = (num_values + threadsPerBlock.x - 1) / threadsPerBlock.x;

    cut::dev_ptr<T> d_values(num_values);
    d_values.loadFromHost(values, num_values);
    cut::dev_ptr<I> d_indices(num_values);
    d_indices.loadFromHost(indices, num_values);
    cut::dev_ptr<T> d_temp(num_values);

    reorderingKernel<<<numPartBlocks, threadsPerBlock>>>(d_values.get(), d_temp.get(), d_indices.get(), num_values);
    d_temp.copyToHost(values, num_values);
}

void reorder_by_index(float *values, const uint32_t *indices, size_t num_values)
{
    reorderByIndexHelper(values, indices, num_values);
}

void reorder_by_index(float *values, const uint64_t *indices, size_t num_values)
{
    reorderByIndexHelper(values, indices, num_values);
}

void reorder_by_index(uint32_t *values, const uint32_t *indices, size_t num_values)
{
    reorderByIndexHelper(values, indices, num_values);
}

void reorder_by_index(uint64_t *values, const uint64_t *indices, size_t num_values)
{
    reorderByIndexHelper(values, indices, num_values);
}

} // namespace device
} // namespace cut
