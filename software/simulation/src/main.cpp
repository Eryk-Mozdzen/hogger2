#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>

#include "generators/Lemniscate.hpp"
#include "controllers/JPTD.hpp"
#include "Model.hpp"
#include "Sink.hpp"

int main() {
	drake::systems::DiagramBuilder<double> builder;

	auto generator = builder.AddSystem<Lemniscate>(2, 10);
	auto controller = builder.AddSystem<JPTD>(1, 10);
	auto model = builder.AddSystem<Model>();
	auto sink = builder.AddSystem<Sink>();

	builder.Connect(generator->get_output_port(), controller->get_trajectory_input_port());
	builder.Connect(model->get_state_output_port(), controller->get_state_input_port());
	builder.Connect(controller->get_control_output_port(), model->get_control_input_port());

	sink->Connect(&builder, model->get_state_output_port(), "x,y,z,phi1,theta1,psi2,theta2");
	sink->Connect(&builder, controller->get_control_output_port(), "dphi1,dtheta1,dphi2,dtheta2");
	sink->Connect(&builder, generator->get_output_port(), {0, 1, 2}, "xd,yd,thetad");

	auto diagram = builder.Build();

	drake::systems::Simulator simulator(*diagram);
	simulator.Initialize();
	simulator.AdvanceTo(30);
}
