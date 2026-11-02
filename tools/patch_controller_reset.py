"""Add per-environment controller reset to the installed rlPx4Controller.

Idempotent source patch, needed because the upstream Python binding retains
rate-integrator state across independent training episodes. No gains change.
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
        header.write_text(h)
    if 'pyParallelRateControl::reset_idx' not in b:
        needle='        .def("set_q_world", &pyParallelRateControl::set_q_world, py::arg("q_world").none())'
        if needle not in b:raise RuntimeError('Unrecognized controller binding')
        b=b.replace('#include <pybind11/pybind11.h>','#include <pybind11/pybind11.h>\n#include <pybind11/stl.h>',1)
        b=b.replace(needle,'        .def("reset_idx", &pyParallelRateControl::reset_idx)\n'+needle,1)
        binding.write_text(b)
    subprocess.run([sys.executable,'-m','pip','install','-e',str(root),'--no-deps','--no-build-isolation'],check=True)
    print('Installed per-environment controller reset:',root)


if __name__=='__main__':main()
