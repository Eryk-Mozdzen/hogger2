#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/analysis/simulator.h>

#include "generators/Lemniscate.hpp"
#include "controllers/JPTD.hpp"
#include "controllers/Smoother.hpp"
#include "Model.hpp"
#include "Sink.hpp"

int main() {
	drake::systems::DiagramBuilder<double> builder;

	auto generator = builder.AddSystem<Lemniscate>(2, 10);
	auto smoother = builder.AddSystem<Smoother>(0, 0, 0);
	auto controller = builder.AddSystem<JPTD>(5, 10);
	auto model = builder.AddSystem<Model>(0, 0, 0);
	auto sink = builder.AddSystem<Sink>();

	builder.Connect(generator->get_output_port(), smoother->get_input_port());
	builder.Connect(smoother->get_output_port(), controller->get_trajectory_input_port());
	builder.Connect(model->get_state_output_port(), controller->get_state_input_port());
	builder.Connect(controller->get_control_output_port(), model->get_control_input_port());

	sink->Connect(&builder, model->get_state_output_port(), {0, 1, 2, 3, 4, 6, 7}, "x,y,theta,phi1,theta1,phi2,theta2");
	sink->Connect(&builder, controller->get_control_output_port(), "eta1,eta2,eta3,eta4,eta5");
	sink->Connect(&builder, generator->get_output_port(), {0, 1, 2}, "xd,yd,thetad");
	sink->Connect(&builder, smoother->get_output_port(), {0, 1, 2}, "xs,ys,thetas");

	auto diagram = builder.Build();

	drake::systems::Simulator simulator(*diagram);

	simulator.get_mutable_integrator().request_initial_step_size_target(1e-7);
	simulator.get_mutable_integrator().set_requested_minimum_step_size(1e-7);
	simulator.get_mutable_integrator().set_throw_on_minimum_step_size_violation(false);
	//simulator.get_mutable_integrator().set_fixed_step_mode(true);

	simulator.Initialize();
	simulator.AdvanceTo(10);
}
