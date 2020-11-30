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
#ifndef CUT_RAYTRACING_H
#define CUT_RAYTRACING_H

#include "cut_common.h"
#include "cut_math.h"
#include "matrix.h"

namespace cut
{

struct Ray
{
    cut::Vec3f origin;
    cut::Vec3f dir;
};

FUNC Ray get_eye_ray_persp(cut::Vec3f cam_pos, cut::Mat4f cam_transform, float cam_aspect, float cam_scale, Vec2f ndc)
{
    Ray ray;
    ray.origin = cam_pos;

    float x = ndc.x * cam_aspect * cam_scale;
    float y = ndc.y * cam_scale;

    auto dir = cam_transform * Vec4f(x, y, -1.0f, 0.0f);
    ray.dir = normalize(cut::Vec3f(dir.x, dir.y, dir.z));
    return ray;
}

FUNC Ray get_eye_ray_ortho(cut::Vec3f cam_pos, cut::Vec3f cam_right, cut::Vec3f cam_up, cut::Vec3f cam_dir, cut::Vec2f ndc)
{
    Ray ray;
    ray.origin = cam_pos + ndc.x * cam_right + ndc.y * cam_up;
    ray.dir = cam_dir;
    return ray;
}

FUNC bool intersect_AABB(Ray r, cut::Vec3f bb_min, cut::Vec3f bb_max, float &t_near, float &t_far)
{
    // compute intersection of ray with all six bbox planes
    cut::Vec3f invR = div(cut::Vec3f{1.0f}, r.dir);
    cut::Vec3f tbot = mul(invR, (bb_min - r.origin));
    cut::Vec3f ttop = mul(invR, (bb_max - r.origin));

    // re-order intersections to find smallest and largest on each axis
    cut::Vec3f tmin = min(ttop, tbot);
    cut::Vec3f tmax = max(ttop, tbot);

    // find the largest tmin and the smallest tmax
    float largest_tmin = max(max(tmin.x, tmin.y), max(tmin.x, tmin.z));
    float smallest_tmax = min(min(tmax.x, tmax.y), min(tmax.x, tmax.z));

    t_near = largest_tmin;
    t_far = smallest_tmax;

    return (smallest_tmax > largest_tmin);
}

} // namespace cut

#endif // RAYTRACING_H
