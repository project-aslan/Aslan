/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <pylon_camera/binary_exposure_search.h>

namespace pylon_camera
{

BinaryExposureSearch::BinaryExposureSearch(const float& target_brightness,
                                           const float& left_lim,
                                           const float& right_lim,
                                           const float& current_exp)
    : last_exposure_(current_exp)
    , last_unchanged_exposure_counter_(0)
    , left_limit_(left_lim)
    , right_limit_(right_lim)
    , new_exposure_((left_lim + right_lim) / 2.0)
    , target_brightness_(target_brightness)
    , limit_reached_(false)
    , is_initial_setting_(true)
{}

BinaryExposureSearch::~BinaryExposureSearch()
{}

bool BinaryExposureSearch::update(const float& current_brightness,
                                  const float& current_exposure)
{
    if ( is_initial_setting_ )
    {
        // no need to update the limits, the first time this function will
        // be called because limits were correctly set in the constructor
        is_initial_setting_ = false;
        return true;
    }

    if ( current_brightness > target_brightness_ )
    {
        right_limit_ = current_exposure;
    }
    else
    {
        left_limit_ = current_exposure;
    }

    new_exposure_ = (left_limit_ + right_limit_) / 2.0;

    if ( new_exposure_ == current_exposure )
    {
       ++last_unchanged_exposure_counter_;
    }
    else
    {
       last_exposure_ = current_exposure;
    }

    if ( last_unchanged_exposure_counter_ > 2 )
    {
        ROS_ERROR_STREAM("BinaryExposureSearch failed, trying three times "
                << "to set the same new exposure value");
        return false;
    }
    else
    {
        return true;
    }
}

const float& BinaryExposureSearch::newExposure() const
{
    return new_exposure_;
}

void BinaryExposureSearch::limitReached(bool reached)
{
    limit_reached_ = reached;
}

bool BinaryExposureSearch::isLimitReached() const
{
    return limit_reached_;
}

}  // namespace pylon_camera
