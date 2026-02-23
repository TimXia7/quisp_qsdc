package modules.Application;
@namespace(quisp::modules);

simple QSDCApplication
{
  parameters:
    @display("i=block/app");
    int number_of_bellpair;
    int bob_addr = default(1);

  gates:
    input fromRouter @loose;
    output toRouter @loose;
}