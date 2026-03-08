package networks;

import modules.QNode;
import modules.Backend.Backend;
import modules.Logger.Logger;
import modules.SharedResource.SharedResource;
import channels.ClassicalChannel;
import channels.QuantumChannel;

network qsdc_network
{
    parameters:
        @display("bgb=300,220");

    submodules:
        backend: Backend { @display("p=40,40"); }
        logger: Logger { @display("p=40,90"); }
        sharedResource: SharedResource { @display("p=40,140"); }

        alice: QNode {
            parameters:
                address = 0;
                node_type = "EndNode";
                @display("p=160,60;i=COMP");
        }

        bob: QNode {
            parameters:
                address = 1;
                node_type = "EndNode";
                @display("p=250,120;i=COMP");
        }

    connections:
        alice.port++ <--> ClassicalChannel { distance = 1km; } <--> bob.port++;
        alice.quantum_port_receiver++ <--> QuantumChannel { distance = 1km; } <--> bob.quantum_port++;

        alice.appQuantumOut --> QuantumChannel {
            distance = 1km;
            channel_loss_rate = 0;
            channel_x_error_rate = 0;
            channel_y_error_rate = 0;
            channel_z_error_rate = 0;
        } --> bob.appQuantumIn;
    }




