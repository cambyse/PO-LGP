#ifndef GRAPHPLOTTING_H_
#define GRAPHPLOTTING_H_

#include <QString>
#include <QTextStream>
#include <QFile>

#include <lemon/core.h>

#include <util/util.h>
#include <util/pretty_printer.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

namespace util {

    template <class GRAPH_T,
        class STRING_1_T = QString,
        class STRING_2_T = QString,
        class STRING_3_T = QString,
        class STRING_4_T = QString>
        void graph_to_pdf(const char* file_name,
                          const GRAPH_T & graph,
                          const STRING_2_T general_node_properties = "",
                          const typename GRAPH_T::template NodeMap<STRING_1_T> * node_properties = nullptr,
                          const STRING_4_T general_arc_properties = "",
                          const typename GRAPH_T::template ArcMap<STRING_3_T> * arc_properties = nullptr) {

        //-------------------//
        // random file names //
        //-------------------//
        QString dot_file_name = util::random_alpha_num(50)+".dot";
        //QString dot_file_name = QString("%1%2").arg(file_name).arg(".dot");
        QString graphics_file_name = file_name;

        //------------------//
        // the file content //
        //------------------//
        // start graph
        QString dot("digraph G {\n");
        // add nodes
        for(typename GRAPH_T::NodeIt node(graph); node!=lemon::INVALID; ++node) {
            auto properties = QString("%1 %2").arg(general_node_properties).arg(node_properties?(*node_properties)[node]:"").simplified();
            dot += QString("\tn%1%2\n").
                arg(graph.id(node)).
                arg(properties==""?"":"["+properties+"]");
        }
        // add arcs
        for(typename GRAPH_T::ArcIt arc(graph); arc!=lemon::INVALID; ++arc) {
            typename GRAPH_T::Node source = graph.source(arc);
            typename GRAPH_T::Node target = graph.target(arc);
            auto properties = QString("%1 %2").arg(general_arc_properties).arg(arc_properties?(*arc_properties)[arc]:"").simplified();
            dot += QString("\tn%1 -> n%2%3\n").
            arg(graph.id(source)).
            arg(graph.id(target)).
            arg(properties==""?"":"["+properties+"]");
        }
        dot += "}";

        //-----------------------------//
        // create file and write to it //
        //-----------------------------//
        {
            QFile outfile(dot_file_name);
            if(!outfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                DEBUG_ERROR("Could not open file");
            }
            QTextStream dot_file_stream(&outfile);
            dot_file_stream << dot;
        }

        //------------------------------//
        // call dot to generate graphic //
        //------------------------------//
        system(QString("dot -Tpdf -o %1 %2").arg(graphics_file_name).arg(dot_file_name).toLatin1());

        //-----------------//
        // remove dot file //
        //-----------------//
        remove(dot_file_name.toLatin1());
    }

}; // end namespace graph_plotting

#include <util/debug_exclude.h>

#endif /* GRAPHPLOTTING_H_ */
