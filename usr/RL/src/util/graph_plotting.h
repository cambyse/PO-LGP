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

    /** Plots a graph to a PDF file. An example of how to use this function
     * would be
     *
     * @code
     * #include <lemon/list_graph.h>
     * #include <util/graph_plotting.h>
     * @endcode
     *
     * \snippet ./unit_tests.cpp graph_to_pdf example
     *
     * which results in this PDF
     *
     * \htmlonly <img src="graph.png" width="200pt"/> \endhtmlonly
     *
     * @tparam GRAPH_T The type of the graph that is used. This should be a
     * directed graph conforming to the lemon::concepts::Digraph concept.
     *
     * @tparam STRING_1_T, STRING_2_T, STRING_3_T, STRING_4_T Types of the
     * different string arguments (or maps of strings).
     *
     * @param file_name Name of the file that will be created.
     *
     * @param graph The graph that will be plotted.
     *
     * @param general_node_properties A string that will be added to all node
     * properties.
     *
     * @param node_properties Pointer to a map with strings that will be added
     * to the properties of individual nodes.
     *
     * @param general_arc_properties A string that will be added to all arc
     * properties.
     *
     * @param arc_properties Pointer to a map with strings that will be added to
     * the properties of individual nodes.
     *
     * @param delete_dot_file Whether to delete the dot file that is created at
     * an intermediate step. The name of the dot file will be a 50 character
     * random string with '.dot' appended if it is deleted afterwards or \p
     * file_name with '.dot' appended if \p delete_dot_file is \c false.
     *
     * */
    template <class GRAPH_T,
        class STRING_1_T = QString,
        class STRING_2_T = QString,
        class STRING_3_T = QString,
        class STRING_4_T = QString,
        class STRING_5_T = QString,
        class STRING_6_T = QString>
        void plot_graph(const char* file_name,
                        const GRAPH_T & graph,
                        const STRING_1_T general_node_properties = "",
                        const typename GRAPH_T::template NodeMap<STRING_2_T> * node_properties = nullptr,
                        const STRING_3_T general_arc_properties = "",
                        const typename GRAPH_T::template ArcMap<STRING_4_T> * arc_properties = nullptr,
                        bool delete_dot_file = true,
                        const STRING_5_T command = "dot",
                        const STRING_6_T parameters = "-Tpdf") {

        //-------------------//
        // random file names //
        //-------------------//
        QString dot_file_name(delete_dot_file?
                              util::random_alpha_num(50)+".dot":
                              QString("%1%2").arg(file_name).arg(".dot"));
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
        system(QString("%1 %2 -o %3 %4").
               arg(command).
               arg(parameters).
               arg(graphics_file_name).
               arg(dot_file_name).toLatin1());

        //-----------------//
        // remove dot file //
        //-----------------//
        if(delete_dot_file) remove(dot_file_name.toLatin1());
    }

}; // end namespace graph_plotting

#include <util/debug_exclude.h>

#endif /* GRAPHPLOTTING_H_ */
