package modules.Application;
@namespace(quisp::modules);

simple QSDCApplication
{
  parameters:
    @display("i=block/app");
    int number_of_bellpair;
    int bob_addr = default(1);

    // Sampling controls (with defaults)
    int min_pairs_to_start = default(8);
    int sample_target = default(32);
    bool expect_anti = default(false);

    double start_delay @unit(s) = default(0s);
    double poll_interval @unit(s) = default(0s);
    double sample_interval @unit(s) = default(0s);

    double max_error_rate = default(0.1);
	string payload = default("hello world");

  gates:
    input fromRouter @loose;
    output toRouter @loose;

    input fromQuantum @loose;
    output toQuantum @loose;
}