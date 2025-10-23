#include <pybind11/pybind11.h>
namespace py = pybind11;

// forward: definita nei .cpp della repo di Sabrina
double r(double W, double V, double nI, double bI, double cI, double T_max, double nF);

PYBIND11_MODULE(gp_lat, m) {
  m.doc() = "Bindings for Sabrina's lateral r(...)";
  m.def("r", &r, py::arg("W"), py::arg("V"),
             py::arg("nI"), py::arg("bI"), py::arg("cI"),
             py::arg("T_max"), py::arg("nF"));
}
