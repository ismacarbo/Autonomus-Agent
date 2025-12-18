#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <rerun/archetypes/scalars.hpp>


#include "action_selection.h"

#include <rerun.hpp>
#include <memory>
#include <string>

namespace py = pybind11;

double r(double W, double V, double nI, double bI, double cI, double T_max, double nF);

extern bool WRITE;
extern bool cortex_rerun;

static std::unique_ptr<rerun::RecordingStream> g_rec;
static int64_t g_step = 0;


static void ensure_rec(const std::string& app_id = "gp_lat") {
    if (!g_rec) {
        g_rec = std::make_unique<rerun::RecordingStream>(app_id);
        g_rec->set_global();        
        g_rec->set_thread_local();
        g_step = 0;
    } else {
        g_rec->set_thread_local();
    }
}

static void rr_spawn(const std::string& app_id = "gp_lat") {
    ensure_rec(app_id);
    
    g_rec->spawn().exit_on_failure();
}

static void rr_connect(const std::string& url,
                       const std::string& app_id = "gp_lat") {
    ensure_rec(app_id);
    auto err = g_rec->connect_grpc(url);
    if (err.is_err()) {
        throw std::runtime_error(std::string("rerun connect_grpc failed: ") + err.description);
    }
}

static void rr_shutdown() {
    g_rec.reset();
    g_step = 0;
}

static void rr_set_time_step(int64_t step) {
    if (g_rec) g_rec->set_time_sequence("step", step);
}


py::tuple sel_jr_py(bool msprt,
                    double end_sim, double goal, double la,
                    double veh_W, double W,
                    const states& k0, const states& k1,
                    double T_max, double V_max,
                    double lat_tol, double tol_obst,
                    double k_dot)
{
    std::vector<double> jr = sel_jr(msprt, end_sim, goal, la, veh_W, W,
                                    k0, k1, T_max, V_max, lat_tol, tol_obst, k_dot);
    double j = (jr.size() > 0) ? jr[0] : 0.0;
    double rr = (jr.size() > 1) ? jr[1] : 0.0;

    
    if (cortex_rerun && g_rec) {
        g_rec->set_time_sequence("step", g_step++);

        g_rec->log("cmd/j", rerun::archetypes::Scalars(j));
        g_rec->log("cmd/r", rerun::archetypes::Scalars(rr));

        g_rec->log("k0/x", rerun::archetypes::Scalars(k0.x));
        g_rec->log("k0/v", rerun::archetypes::Scalars(k0.v));
        g_rec->log("k0/a", rerun::archetypes::Scalars(k0.a));
        g_rec->log("k0/n", rerun::archetypes::Scalars(k0.n));
        g_rec->log("k0/b", rerun::archetypes::Scalars(k0.b));
        g_rec->log("k0/c", rerun::archetypes::Scalars(k0.c));

        g_rec->log("k1/x", rerun::archetypes::Scalars(k1.x));
        g_rec->log("k1/v", rerun::archetypes::Scalars(k1.v));
        g_rec->log("k1/a", rerun::archetypes::Scalars(k1.a));
        g_rec->log("k1/n", rerun::archetypes::Scalars(k1.n));
        g_rec->log("k1/b", rerun::archetypes::Scalars(k1.b));
        g_rec->log("k1/c", rerun::archetypes::Scalars(k1.c));

    }

    return py::make_tuple(j, rr);
}

PYBIND11_MODULE(gp_lat, m) {
    m.doc() = "Bindings for Sabrina's action selection (sel_jr) + optional Rerun logging";

    m.def("r", &r);

    m.def("set_write_csv", [](bool v){ WRITE = v; }, py::arg("enabled"));

    
    m.def("set_cortex_rerun", [](bool v){ cortex_rerun = v; }, py::arg("enabled"));

    py::class_<states>(m, "states")
      .def(py::init<>())
      .def_readwrite("x", &states::x)
      .def_readwrite("v", &states::v)
      .def_readwrite("a", &states::a)
      .def_readwrite("n", &states::n)
      .def_readwrite("b", &states::b)
      .def_readwrite("c", &states::c);

    
    m.def("rerun_spawn", &rr_spawn, py::arg("app_id") = "gp_lat");
    m.def("rerun_connect", &rr_connect,
          py::arg("url") = "rerun+http:
          py::arg("app_id") = "gp_lat");
    m.def("rerun_shutdown", &rr_shutdown);
    m.def("rerun_set_time_step", &rr_set_time_step, py::arg("step"));

    m.def("sel_jr", &sel_jr_py,
          py::arg("msprt"),
          py::arg("end_sim"),
          py::arg("goal"),
          py::arg("la"),
          py::arg("veh_W"),
          py::arg("W"),
          py::arg("k0"),
          py::arg("k1"),
          py::arg("T_max"),
          py::arg("V_max"),
          py::arg("lat_tol"),
          py::arg("tol_obst"),
          py::arg("k_dot"));
}
