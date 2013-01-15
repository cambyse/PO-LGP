/** @mainpage Biros

Biros is a communication framework for application in robotics.


@section intro_biros Introduction

The basic components of Biros are:
- A Variable is a dump object which holds some data from a calculation.
- A Process does some calculations and exchanges information via Varibales.
- Parameter TODO

To create your own Processes and/or Variables you simply have to inherit from Process and/or Variable.
Variables and Processes create a graph.

The actual class Biros allows control and access about the entire graph of variables and processes.

Most of the core structures are in defined in biros.h.


@section biros_examples Examples

Examples can be found under share/src/biros*
*/
