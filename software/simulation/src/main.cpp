#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>

#include "generators/Lemniscate.hpp"
#include "controllers/JPTD.hpp"
#include "Model.hpp"
#include "Sink.hpp"

int main() {
	drake::systems::DiagramBuilder<double> builder;

	auto generator = builder.AddSystem<Lemniscate>(2, 10);
	auto controller = builder.AddSystem<JPTD>(2, 1);
	auto model = builder.AddSystem<Model>(0, 0, 0);
	auto sink = builder.AddSystem<Sink>();

	builder.Connect(generator->get_output_port(), controller->get_trajectory_input_port());
	builder.Connect(model->get_state_output_port(), controller->get_state_input_port());
	builder.Connect(controller->get_control_output_port(), model->get_control_input_port());

	sink->Connect(&builder, model->get_state_output_port(), {0, 1, 2, 3, 4, 6, 7}, "x,y,theta,phi1,theta1,phi2,theta2");
	sink->Connect(&builder, controller->get_control_output_port(), "dphi1,dtheta1,dpsi1,dtheta2,dpsi2");
	sink->Connect(&builder, generator->get_output_port(), {0, 1, 2}, "xd,yd,thetad");

	auto diagram = builder.Build();

	drake::systems::Simulator simulator(*diagram);

	//simulator.get_mutable_integrator().request_initial_step_size_target(1e-6);
	//simulator.get_mutable_integrator().set_requested_minimum_step_size(1e-6);
	//simulator.get_mutable_integrator().set_maximum_step_size(1e-5);
	//simulator.get_mutable_integrator().set_throw_on_minimum_step_size_violation(false);
	//simulator.get_mutable_integrator().set_fixed_step_mode(true);

	simulator.Initialize();
	simulator.AdvanceTo(20);
}
