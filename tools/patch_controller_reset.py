"""Add per-environment controller reset to the installed rlPx4Controller.

Idempotent source patch: independent reset and configurable rate gains.
Default gains remain unchanged; configured gains survive episode resets.
"""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path)
    args=parser.parse_args()
    root=args.root
    if root is None:
        spec=importlib.util.find_spec('rlPx4Controller')
        if spec is None:
            raise RuntimeError('Install rlPx4Controller first')
        root=Path(spec.origin).resolve().parents[1]
    header=root/'include/parallelRateController.hpp'
    binding=root/'bind/pyParallelControl.cpp'
    h=header.read_text();b=binding.read_text()
    if 'void reset_idx(' not in h:
        needle='    pyParallelRateControl(int envs_num);'
        if needle not in h:raise RuntimeError('Unrecognized controller header')
        h=h.replace(needle,needle+'''
    void reset_idx(const std::vector<int> &indices)
    {
        for (int index : indices)
        {
            _rate_control.at(index) = Px4RateController();
            _mixers.at(index) = Px4Mixer();
        }
    }
''',1)
    if 'void set_pid_params(' not in h:
        needle='    pyParallelRateControl(int envs_num);'
        h=h.replace(needle,'''    Eigen::Vector3d _configured_p = Eigen::Vector3d(0.5, 0.5, 0.2);
    Eigen::Vector3d _configured_i = Eigen::Vector3d(0.08, 0.08, 0.05);
    Eigen::Vector3d _configured_d = Eigen::Vector3d(0.001, 0.001, 0.0);
    void set_pid_params(Eigen::Vector3d p, Eigen::Vector3d i, Eigen::Vector3d d)
    {
        _configured_p = p; _configured_i = i; _configured_d = d;
        for (auto &controller : _rate_control) controller.set_pid_params(p, i, d);
    }
'''+needle,1)
        h=h.replace('_rate_control.at(index) = Px4RateController();',
            '_rate_control.at(index) = Px4RateController();\n'
            '            _rate_control.at(index).set_pid_params(_configured_p, _configured_i, _configured_d);',1)
    if 'pyParallelRateControl::reset_idx' not in b:
        needle='        .def("set_q_world", &pyParallelRateControl::set_q_world, py::arg("q_world").none())'
        if needle not in b:raise RuntimeError('Unrecognized controller binding')
        b=b.replace('#include <pybind11/pybind11.h>','#include <pybind11/pybind11.h>\n#include <pybind11/stl.h>',1)
        b=b.replace(needle,'        .def("reset_idx", &pyParallelRateControl::reset_idx)\n'+needle,1)
    if 'pyParallelRateControl::set_pid_params' not in b:
        needle='        .def("set_q_world", &pyParallelRateControl::set_q_world, py::arg("q_world").none())'
        if needle not in b:raise RuntimeError('Unrecognized controller binding')
        b=b.replace(needle,'        .def("set_pid_params", &pyParallelRateControl::set_pid_params)\n'+needle,1)
    # Upstream uses env index as an Eigen block row count. Use fixed-size rows
    # so different batch indices do not produce mismatched matrix dimensions.
    for old,new in [
        ('q_matrix.block(i, 0, i, 4).transpose()', 'q_matrix.row(i).transpose()'),
        ('actions.block(i, 0, i + 1, 3).transpose()', 'actions.row(i).head<3>().transpose()'),
        ('observations.block(i, 0, i + 1, 3).transpose()', 'observations.row(i).head<3>().transpose()'),
        ('_commands.block(i, 0, i + 1, 4) <<', '_commands.row(i) <<'),
    ]:
        h=h.replace(old,new)
    header.write_text(h);binding.write_text(b)
    subprocess.run([sys.executable,'-m','pip','install','-e',str(root),'--no-deps','--no-build-isolation'],check=True)
    print('Installed per-environment controller reset:',root)


if __name__=='__main__':main()
