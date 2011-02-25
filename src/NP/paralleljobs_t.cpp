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

#ifndef NP_PARALLELJOBS_T_CPP
#define NP_PARALLELJOBS_T_CPP

#include <ctime>
#include "paralleljobs.h"
#include "nputils.h"

template<class S>
np::ParallelJobs<S>::ParallelJobs(const MT::Array<S*>& jobs) : current_index(0),
num_running_threads(0)
{
  jobs_ = jobs;
  pthread_mutex_init(&jobs_mutex_, NULL);
}

template<class S>
np::ParallelJobs<S>::~ParallelJobs()
{
  jobs_.clear();
  pthread_mutex_destroy(&jobs_mutex_);
}

template<class S>
void np::ParallelJobs<S>::run
(
 void* (*callback)(void*),
 uint num_threads,
 bool verbose
)
{
  // don't do anything w/o any list
  if (jobs_.N == 0)
    msg_error(HERE, "Job list is empty, nothing will be started ...");

  // have at least one thread doing the job
  if (num_threads == 0)
    num_threads = 1;

  if (verbose)
    std::cout << "jobs: " << jobs_.N << ", "
              << "threads: " << num_threads
              << ", off we go!" << std::endl;

  // start threads
  uint start = std::clock();
  pthread_t thread_ids[num_threads];
  for (uint ti = 0; ti < num_threads; ti++)
  {
    if (pthread_create(&thread_ids[ti], NULL, callback, this) == 0);
      num_running_threads++;
    if (verbose)
      std::cout << num_running_threads << " threads started ...\r" <<  std::flush;
  }

  if (verbose)
  {
    std::cout << std::endl;
    bool keep_running = true;
    uint runtime;
    while (keep_running)
    {
      sleep(1);
      runtime = std::clock()-start;
      keep_running = (current_index >= jobs_.N ? false : true);
      std::cout << "jobs assigned: " << std::setw(3) << std::setfill('0')
      << (int) ((double) current_index / (double) jobs_.N * 100) << "%"
      /*<< "runtime: " << 0 << "h:" << 0 << "m:" << 0 << "s"*/
      << "\r"
      << std::flush;
    }
    std::cout << std::endl;
  }

  // join pthreads
  for (uint ti = 0; ti < num_threads; ti++)
  {
    if (verbose)
    {
      std::cout << "still " << std::setw(3) << num_running_threads
      << " threads running ...\r" << std::flush;
    }
    pthread_join(thread_ids[ti], NULL);
    num_running_threads--;
  }

  if (verbose)
    std::cout << std::endl << "all jobs done" << std::endl;
}

template<class S>
np::JOB_STATUS np::ParallelJobs<S>::get_job(S& job)
{
  np::JOB_STATUS status = np::JOB_STATUS_ERROR;
  pthread_mutex_lock(&jobs_mutex_);                            // ENTER_CRITICAL

  if (current_index < jobs_.N)                          // something left to do?
  {
    job = *jobs_(current_index);                        // dispatch the next job
    current_index++;
    status = np::JOB_STATUS_OK;
  }
  else
    status = JOB_STATUS_EMPTY;

  pthread_mutex_unlock(&jobs_mutex_);                          // LEAVE_CRITICAL
  return status;
}
#endif