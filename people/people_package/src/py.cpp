#include "Python.h"

#include <stdio.h>
#include <iostream>

#include "ros/node.h"
#include "deprecated_msgs/ImageArray.h"
#include "std_msgs/String.h"
#include "image_utils/cv_bridge.h"
#include "CvStereoCamModel.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "people.h"

#include "generated.i"

PyObject *wrapped_People_detectAllFaces(PyObject *self, PyObject *args)
{
  wrapped_People_t *pp = (wrapped_People_t*)self;

  // IplImage *image, const char* haar_classifier_filename, double threshold, IplImage *disparity_image, CvStereoCamModel *cam_model, bool do_draw)
  IplImage *input;
  char *haar_filename;
  double threshold;
  PyObject *disparity_image, *cam_model;
  int do_draw;

  {
    char *imgdata;
    int imgdata_size, x, y;
    if (!PyArg_ParseTuple(args, "s#iisdOOi", &imgdata, &imgdata_size, &x, &y, &haar_filename, &threshold, &disparity_image, &cam_model, &do_draw))
      return NULL;
    input = cvCreateImageHeader(cvSize(x, y), IPL_DEPTH_8U, 1);
    cvSetData(input, imgdata, x);
  }

  vector<CvRect> faces_vector = pp->c.detectAllFaces(input, haar_filename, threshold, NULL, NULL, do_draw);
  PyObject *r = PyList_New(faces_vector.size());
  for (size_t i = 0; i < faces_vector.size(); i++)
    PyList_SetItem(r, i, Py_BuildValue("iiii",
                    faces_vector[i].x,
                    faces_vector[i].y,
                    faces_vector[i].width,
                    faces_vector[i].height));
  return r;
}

static PyMethodDef methods[] = {
  {"people", make_wrapped_People, METH_VARARGS},
  {NULL, NULL},
};

extern "C" void initface_detection()
{
    PyObject *m, *d;

    m = Py_InitModule("face_detection", methods);
    d = PyModule_GetDict(m);
}
