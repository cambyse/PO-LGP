/*
 * NodeStateModel.h
 *
 *  Created on: May 3, 2012
 *      Author: robert
 */

#ifndef NODESTATEMODEL_H_
#define NODESTATEMODEL_H_

#include "GraphModel.h"

class NodeStateModel {

public:

	NodeStateModel(): node(INVALID) {};
	NodeStateModel(const Node& n): node(n) {};

	virtual ~NodeStateModel() {}

	Node get_node() const { return node; }
	void set_node(const Node& n) { node = n; }

	bool virtual operator==(const NodeStateModel& other) const { return node==other.get_node(); }
	bool virtual operator!=(const NodeStateModel& other) const { return !(*this==other); }

private:

	Node node;

};

#endif /* NODESTATEMODEL_H_ */
