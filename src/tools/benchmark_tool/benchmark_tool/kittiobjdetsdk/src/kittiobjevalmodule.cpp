// Copyright (c) 2020-2021, Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define PY_SSIZE_T_CLEAN
#include <common/types.hpp>
#include <Python.h>
#include <cstdio>

#include <iostream>
#include <string>

#include "kittiobjevalmodule.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

static PyObject *
kittiobjeval_eval(PyObject *, PyObject * args)
{
  const char8_t * ground_truth_path;
  const char8_t * detection_path;
  const char8_t * output_path;
  bool8_t eval_2d_res;
  bool8_t eval_ground_res;
  bool8_t eval_3d_res;
  bool8_t print_stdout = false;
  bool8_t create_plot = false;
  int32_t sts;

  if (!PyArg_ParseTuple(
      args, "sssbbb|bb", &ground_truth_path,
      &detection_path, &output_path, &eval_2d_res, &eval_ground_res,
      &eval_3d_res, &print_stdout, &create_plot) )
  {
    return NULL;
  }
  sts = kittisdk::eval(
    std::basic_string<char8_t>(ground_truth_path),
    std::basic_string<char8_t>(detection_path),
    std::basic_string<char8_t>(output_path),
    eval_2d_res,
    eval_ground_res,
    eval_3d_res,
    print_stdout,
    create_plot);
  return PyBool_FromLong(sts);
}


static PyMethodDef kittiobjevalMethods[] = {
  {"eval", kittiobjeval_eval, METH_VARARGS,
    "Evaluate using kitti object detection evaluation."},
  {NULL, NULL, 0, NULL}          /* Sentinel */
};


static struct PyModuleDef kittiobjeval_module = {
  PyModuleDef_HEAD_INIT,
  "kittiobjeval",       // name of module
  NULL,                 // module documentation, may be NULL
  -1,                   // size of per-interpreter state of the module,
                        // or -1 if the module keeps state in global variables.
  kittiobjevalMethods,
  NULL,
  NULL,
  NULL,
  NULL,
};

extern "C" KITTIOBJEVALMODULE_PUBLIC PyObject * PyInit_kittiobjeval(void)
{
  return PyModule_Create(&kittiobjeval_module);
}
