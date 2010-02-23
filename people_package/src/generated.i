
typedef struct {
    PyObject_HEAD
    People 
#if 0
    *
#endif
    c; 
} wrapped_People_t;

static void
wrapped_People_dealloc(PyObject *self)
{
#if 0
  wrapped_People_t *pc = (wrapped_People_t*)self;
  delete pc->c;
#endif
  PyObject_Del(self);
}

PyObject *wrapped_People_detectAllFaces(PyObject *self, PyObject *args);

/* Method table */
static PyMethodDef wrapped_People_methods[] = {
{"detectAllFaces", wrapped_People_detectAllFaces, METH_VARARGS},
{NULL, NULL}
};

static PyObject *
wrapped_People_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(wrapped_People_methods, self, attrname);
}

static PyTypeObject wrapped_People_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "People",
    sizeof(wrapped_People_t),
    0,
    (destructor)wrapped_People_dealloc,
    0,
    (getattrfunc)wrapped_People_GetAttr,
    0,
    0,
    0, // repr
    0,
    0,
    0,

    0,
    0,
    0,
    0,
    0,
    
    0,
    
    Py_TPFLAGS_CHECKTYPES,

    0,
    0,
    0,
    0

    /* the rest are NULLs */
};

PyObject *make_wrapped_People(PyObject *self, PyObject *args)
{
    wrapped_People_t *object = PyObject_NEW(wrapped_People_t, &wrapped_People_Type);
#if 0
    object->c = new People ;
#else
    new(&object->c) People();
#endif
    return (PyObject*)object;
}

