////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////


//-----------------------------------------------------------------------------
/*!\file    PDController.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2019/10/16
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/PDController.h>
#include <cmath> 

namespace cartesian_controller_base
{

PDController::PDController()
  : m_last_p_error(0.0)
{
}

PDController::~PDController()
{
}


void PDController::init(const std::string& params, std::shared_ptr<rclcpp::Node> handle)
{
  m_params = params;
  m_handle = handle;

  auto auto_declare = [this](const std::string& s)
  {
    if (!m_handle->has_parameter(s))
    {
      return m_handle->declare_parameter<double>(s, 0.0);
    }
    else
    {
      return m_handle->get_parameter(s).as_double();
    }
  };

  auto_declare(m_params + ".p");
  auto_declare(m_params + ".d");

  m_handle->declare_parameter<double> (m_params + ".error_filter.cutoff_freq", -1);
  m_handle->declare_parameter<bool>( m_params + ".error_filter.use_real_update_rate", false);
}


double PDController::operator()(const double& error, const rclcpp::Duration& period)
{
  if (period == rclcpp::Duration::from_nanoseconds(0.0))
  {
    return 0.0;
  }
 
  // Get latest gains
  m_handle->get_parameter(m_params + ".p", m_p);
  m_handle->get_parameter(m_params + ".d", m_d);

  // Get filter configuration
  //TODO cutoff_freq and use_real_update_rate as class members
  double cutoff_freq;
  m_handle->get_parameter(m_params + ".error_filter.cutoff_freq", cutoff_freq);
  bool use_real_update_rate;
  m_handle->get_parameter(m_params + ".error_filter.use_real_update_rate", use_real_update_rate);

  // Filter error
  double filtered_error = error;
  if (cutoff_freq > 0) // Filter disactivated for cutoff_freq <= 0
  {
    double dt = use_real_update_rate == true ? 
      1.0/m_handle->get_parameter("update_rate").as_double()
      : period.seconds();
    double alpha = 1 - exp(-dt*2*M_PI*cutoff_freq);
    filtered_error = alpha * error + (1- alpha) * m_last_p_error;
  }

  // Copute PD output
  double result = m_p * filtered_error + m_d * (filtered_error - m_last_p_error) / period.seconds();

  m_last_p_error = error;
  return result;
}


}
