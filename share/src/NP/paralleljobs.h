/*  Copyright 2009 Nils Plath
    email: nilsp@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/> */

/** \file paralleljobs.h
    \brief Parallelize a simple routine using multiple threads */

#ifndef NP_PARALLELJOBS_H
#define NP_PARALLELJOBS_H

#include <MT/array.h>
#include <pthread.h>

namespace np
{

typedef enum
{
  JOB_STATUS_OK,
  JOB_STATUS_EMPTY,
  JOB_STATUS_ERROR
} JOB_STATUS;

template<class S>
class ParallelJobs
{
  public:
                        ParallelJobs(const MT::Array<S*>& jobs);
                        ~ParallelJobs();

    void                run
                        (
                         void* (*callback)(void*),
                         uint num_threads = 8,
                         bool verbose = false
                        );
    JOB_STATUS          get_job(S& job);

  private:
    uint                current_index;
    uint                num_running_threads;
    pthread_mutex_t     jobs_mutex_;
    MT::Array<S*>       jobs_;
};
} // namespace np

#if defined NP_IMPLEMENT_TEMPLATES | MT_IMPLEMENT_TEMPLATES
#include "paralleljobs_t.cpp"
#endif

#endif



