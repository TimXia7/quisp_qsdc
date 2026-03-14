#include "QSDCApplication.h"

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <omnetpp.h>

#include "messages/classical_messages.h"
#include "modules/QNIC/StationaryQubit/StationaryQubit.h"

using namespace omnetpp;
using namespace quisp::messages;

namespace quisp::modules {

Define_Module(QSDCApplication);

// separate namespace to isolate logging with macro: QLOG("<log message>");
namespace {
// Logging to qsdc_app.log
inline void qsdc_log(const std::string& msg) {
  static std::ofstream logfile("qsdc_app.log", std::ios::app);
  if (!logfile.is_open()) return;

  logfile << std::fixed << std::setprecision(9)
          << "[" << omnetpp::simTime() << "] "
          << msg << "\n";
  logfile.flush();
}

#define QLOG(expr)                         \
  do {                                     \
    std::ostringstream _qs;                \
    _qs << expr;                           \
    qsdc_log(_qs.str());                   \
  } while (0)

}  // namespace

// const chars (essentially ENUMS) that determine simulation settings:
static const char* SELF_START_ONCE = "START_ONCE";
static const char* SELF_WAIT_FOR_PAIRS = "WAIT_FOR_PAIRS";
static const char* SELF_NEXT_SAMPLE = "NEXT_SAMPLE";
static const char* SELF_NEXT_BELL_CHECK = "NEXT_BELL_CHECK";

// Phase 1
static const char* ENT_RESP = "ENTCHECK_RESP";

// Phase 2
static const char* BELL_REQ = "BELLCHECK_REQ";
static const char* BELL_RESP = "BELLCHECK_RESP";

// Phase 3
static const char* SELF_START_MESSAGE = "START_MESSAGE";
static const char* DENSE_DONE = "DENSE_DONE";
static const char* SAMPLE_PHOTON = "sample_photon";

// Convert eigenvalue result to +/-1
// simplifies vector calculations of qubits
static inline int eigenToInt(quisp::types::EigenvalueResult r) {
  return (r == quisp::types::EigenvalueResult::PLUS_ONE) ? +1 : -1;
}

// Step 1: on program startup, initializes system
void QSDCApplication::initialize() {
  initializeLogger(provider);

  // Only keep this module for EndNodes (same logic as Application.cc)
  // Basically, only end nodes should run this.
  if (!gate("toRouter")->isConnected() || !gate("fromRouter")->isConnected()) {
    auto* msg = new DeleteThisModule("DeleteThisModule");
    scheduleAt(simTime(), msg);
    return;
  }

  // init QNode process cont.
  my_address = provider.getNodeAddr();

  auto* qnode = provider.getQNode();
  if (!qnode) {
    qsdc_log("[QSDC] No QNode found in initialize()");
    return;
  }
  is_initiator = qnode->par("is_initiator").boolValue();

  // Read sampling parameters from NED/INI
  min_pairs_to_start = par("min_pairs_to_start").intValue();
  sample_target = par("sample_target").intValue();
  expect_anti_correlation = par("expect_anti").boolValue();

  eve_enabled = par("eve_enabled").boolValue();
  eve_intercept_probability = par("eve_intercept_probability").doubleValue();

  // Phase 2 Checking
  bell_sample_target = par("bell_sample_target").intValue();
  bell_check_started = false;
  bell_samples_done = 0;
  bell_errors = 0;

  start_delay = par("start_delay");
  poll_interval = par("poll_interval");
  sample_interval = par("sample_interval");
  burn_count = par("burn_count").intValue();
  burn_current = 0;
  protocol_started = false;

  if (is_initiator) {
    scheduleAt(simTime(), new cMessage(SELF_START_ONCE));
  }
}

// Step 2: Initialize schedules SELF_START_ONCE in handleMessage, which schedules startOnce.
// This completes the startup process
void QSDCApplication::startOnce() {
  const int bob_addr = par("bob_addr").intValue();
  const int n_pairs = par("number_of_bellpair").intValue();

  auto* pk = new ConnectionSetupRequest("ConnSetupRequest");
  pk->setApplicationId(0);

  pk->setActual_srcAddr(my_address);
  pk->setActual_destAddr(bob_addr);

  // routing headers
  pk->setSrcAddr(my_address);
  pk->setDestAddr(my_address);

  pk->setNumber_of_required_Bellpairs(n_pairs);
  pk->setNum_measure(0);
  pk->setKind(7);

  QLOG("[QSDC] Requesting " << n_pairs << " stored Bell pairs from "
       << my_address << " to " << bob_addr);

  send(pk, "toRouter");
}

// Step 3: preliminary settings for the actual protocol starting.
// Once a delay is over, the polling starts to check if there are enough pairs to start QSDC
// When there is, bell pair sampling starts
void QSDCApplication::startQSDCProtocol(unsigned long ruleset_id) {
  if (!is_initiator) return;

  if (protocol_started) {
    QLOG("[QSDC] Duplicate ConnectionSetupResponse ignored (ruleset_id="
         << ruleset_id << ")");
    return;
  }
  protocol_started = true;

  active_ruleset_id = ruleset_id;

  samples_done = 0;
  errors = 0;
  sampling_started = false;
  burn_current = 0;
  pending_checks.clear();
  bell_check_started = false;
  bell_samples_done = 0;
  bell_errors = 0;
  pending_bell_checks.clear();

  // Bell-pair usage stays separate for dense coding
  used_indices.clear();

  QLOG("[QSDC] Starting sampling after delay=" << start_delay
       << ", waiting for >= " << min_pairs_to_start << " ready Bell pairs for later dense phase.");

  scheduleAt(simTime() + start_delay, new cMessage(SELF_WAIT_FOR_PAIRS));
}

// Step 4: keep polling after the initial wait until there are enough pairs to initiate the process
// the number of pairs is defined with "min_pairs_to_start", defined in ini
void QSDCApplication::pollUntilEnoughPairs() {
  if (!is_initiator) return;

  std::vector<int> ready;
  const int n_ready = countReadyPairsAndCollect(ready);
  QLOG("[QSDC] Ready slots=" << n_ready << " (need >= " << min_pairs_to_start << ")");

  if (n_ready >= min_pairs_to_start) {
    sampling_started = true;
    scheduleAt(simTime(), new cMessage(SELF_NEXT_SAMPLE));
  } else {
    scheduleAt(simTime() + poll_interval, new cMessage(SELF_WAIT_FOR_PAIRS));
  }
}

// (part of step 4) Calculate the amount of available bell pairs
int QSDCApplication::countReadyPairsAndCollect(std::vector<int>& out_indices) {
  out_indices.clear();

  auto* qnic = getLocalEntangledQnic();
  if (!qnic) return 0;

  const int num_buf = qnic->par("num_buffer").intValue();

  for (int i = 0; i < num_buf; i++) {
    auto* sq_mod = qnic->getSubmodule("statQubit", i);
    if (!sq_mod) continue;

    auto* sq = check_and_cast<quisp::modules::StationaryQubit*>(sq_mod);

    // "Ready" heuristic:
    // - busy: slot currently holds a prepared resource (entangled memory ?)
    // - not locked: available for us
    // - not used: haven't consumed it already by sampling
    if (sq->isBusy() && !sq->isLocked() && used_indices.find(i) == used_indices.end()) {
      out_indices.push_back(i);
    }
  }

  return (int)out_indices.size();
}

// Step 5: Sampling begins; Alice prepares dedicated test qubits in random X/Z bases
// and random bit values, then sends them unmeasured through the same quantum channel
// used later for the dense-coded message. Bob measures each incoming test photon in a
// random basis and reports his basis/result classically. Alice compares only the
// matched-basis cases to estimate the channel error rate.
void QSDCApplication::doNextSample() {
  if (!is_initiator) return;

  if (!sampling_started) {
    scheduleAt(simTime() + poll_interval, new cMessage(SELF_WAIT_FOR_PAIRS));
    return;
  }

  if (samples_done >= sample_target) {
    const double err_rate = (samples_done == 0) ? 0.0 : (double)errors / (double)samples_done;
    QLOG("[QSDC] Sampling finished: samples=" << samples_done
         << " errors=" << errors
         << " error_rate=" << err_rate);

    if (err_rate <= par("max_error_rate").doubleValue()) {
      QLOG("[QSDC] Channel accepted; starting Bell-pair correlation check.");
      startBellCheckPhase();
    } else {
      QLOG("[QSDC] Channel rejected; dense-coded transmission aborted.");
    }
    return;
  }
  std::vector<int> ready;
  const int n_ready = countReadyPairsAndCollect(ready);

  if (n_ready == 0) {
    QLOG("[QSDC] No ready slots right now; polling again");
    scheduleAt(simTime() + poll_interval, new cMessage(SELF_WAIT_FOR_PAIRS));
    return;
  }

  const int qi = ready.back();

  auto* qnic = getLocalEntangledQnic();
  if (!qnic) {
    QLOG("[QSDC] No local qnic found for sampling.");
    return;
  }

  auto* sq_mod = qnic->getSubmodule("statQubit", qi);
  if (!sq_mod) {
    QLOG("[QSDC] statQubit[" << qi << "] not found on local qnic");
    scheduleAt(simTime() + poll_interval, new cMessage(SELF_WAIT_FOR_PAIRS));
    return;
  }

  auto* qubit = check_and_cast<quisp::modules::StationaryQubit*>(sq_mod);

  // Sacrifice this Bell-pair slot for testing:
  // reset to |0>
  resetQubitToZero(qubit);

  const char basis = (dblrand() < 0.5) ? 'Z' : 'X';
  const int bit = (dblrand() < 0.5) ? 0 : 1;

  prepareTestState(qubit, basis, bit);

  pending_checks[qi] = PendingCheck{basis, bit};
  used_indices.insert(qi);

  QLOG("[QSDC] Sample request " << (samples_done + 1)
       << ": qi=" << qi
       << " prepared_basis=" << basis
       << " prepared_bit=" << bit
       << " sending SAMPLE_PHOTON through quantum channel");

  sendSamplePhoton(qi, qubit, basis, bit);
}

void QSDCApplication::resetQubitToZero(quisp::modules::StationaryQubit* qubit) {
  // Force the sacrificial memory qubit into |0>.
  // Measure in Z; if result is |1>, flip it to |0>.
  const int z = eigenToInt(qubit->measureZ());
  if (z == -1) {
    qubit->gateX();
  }
}

void QSDCApplication::prepareTestState(quisp::modules::StationaryQubit* qubit, char basis, int bit) {
  // Starting from |0>, prepare one of:
  // Z basis: |0>, |1>
  // X basis: |+>, |->
  if (basis == 'Z') {
    if (bit == 1) {
      qubit->gateX();  // |1>
    }
  } else if (basis == 'X') {
    if (bit == 0) {
      qubit->gateHadamard();  // |+>
    } else {
      qubit->gateX();
      qubit->gateHadamard();  // |->
    }
  } else {
    throw cRuntimeError("prepareTestState: invalid basis '%c'", basis);
  }
}

void QSDCApplication::sendSamplePhoton(int qi, quisp::modules::StationaryQubit* qubit, char basis, int bit) {
  auto* photon = new quisp::messages::PhotonicQubit("SAMPLE_PHOTON");
  photon->setMessage_type(SAMPLE_PHOTON);

  // This is the important part: send the actual qubit through the channel.
  photon->setQubitRef(qubit->getBackendQubitRef());

  photon->addPar("src_addr") = my_address;
  photon->addPar("qubit_index") = qi;

  send(photon, "toQuantum");

  QLOG("[QSDC] Sample photon sent through quantum channel: qi=" << qi
       << " basis=" << basis
       << " bit=" << bit);
}

int QSDCApplication::measureLocalInBasis(quisp::modules::StationaryQubit* qubit, char basis) {
  if (basis == 'Z') {
    return (eigenToInt(qubit->measureZ()) == +1) ? 0 : 1;
  }
  if (basis == 'X') {
    return (eigenToInt(qubit->measureX()) == +1) ? 0 : 1;
  }
  throw cRuntimeError("measureLocalInBasis: invalid basis '%c'", basis);
}

void QSDCApplication::startBellCheckPhase() {
  bell_check_started = true;
  bell_samples_done = 0;
  bell_errors = 0;
  pending_bell_checks.clear();

  if (bell_sample_target <= 0) {
    QLOG("[QSDC] Bell-correlation check skipped; starting dense-coded message transmission.");
    scheduleAt(simTime() + sample_interval, new cMessage(SELF_START_MESSAGE));
    return;
  }

  QLOG("[QSDC] Starting Bell-pair correlation check with target=" << bell_sample_target);
  scheduleAt(simTime(), new cMessage(SELF_NEXT_BELL_CHECK));
}

void QSDCApplication::sendBellCheckRequest(int qi, char basis) {
  auto* req = new Header(BELL_REQ);
  req->setSrcAddr(my_address);
  req->setDestAddr(par("bob_addr").intValue());
  req->setKind(1);

  req->addPar("qubit_index") = qi;
  req->addPar("basis") = std::string(1, basis).c_str();

  send(req, "toRouter");

  QLOG("[QSDC] BELLCHECK_REQ sent: qi=" << qi << " basis=" << basis);
}

void QSDCApplication::doNextBellCheck() {
  if (!is_initiator) return;

  if (!bell_check_started) {
    scheduleAt(simTime() + poll_interval, new cMessage(SELF_NEXT_BELL_CHECK));
    return;
  }

  if (bell_samples_done >= bell_sample_target) {
    const double bell_err_rate =
        (bell_samples_done == 0) ? 0.0 : (double)bell_errors / (double)bell_samples_done;

    QLOG("[QSDC] Bell-correlation check finished: samples=" << bell_samples_done
         << " errors=" << bell_errors
         << " error_rate=" << bell_err_rate);

    if (bell_err_rate <= par("max_bell_error_rate").doubleValue()) {
      QLOG("[QSDC] Bell pairs accepted; starting dense-coded message transmission.");
      scheduleAt(simTime() + sample_interval, new cMessage(SELF_START_MESSAGE));
    } else {
      QLOG("[QSDC] Bell pairs rejected; dense-coded transmission aborted.");
    }
    return;
  }

  std::vector<int> ready;
  const int n_ready = countReadyPairsAndCollect(ready);

  if (n_ready == 0) {
    QLOG("[QSDC] No ready Bell pairs for Phase 2 right now; polling again");
    scheduleAt(simTime() + poll_interval, new cMessage(SELF_NEXT_BELL_CHECK));
    return;
  }

  const int qi = ready.back();

  auto* qnic = getLocalEntangledQnic();
  if (!qnic) {
    QLOG("[QSDC] No local qnic found for Bell correlation check.");
    return;
  }

  auto* sq_mod = qnic->getSubmodule("statQubit", qi);
  if (!sq_mod) {
    QLOG("[QSDC] Bell check statQubit[" << qi << "] not found on Alice");
    scheduleAt(simTime() + poll_interval, new cMessage(SELF_NEXT_BELL_CHECK));
    return;
  }

  auto* qubit = check_and_cast<quisp::modules::StationaryQubit*>(sq_mod);

  const char basis = (dblrand() < 0.5) ? 'Z' : 'X';
  const int alice_bit = measureLocalInBasis(qubit, basis);

  pending_bell_checks[qi] = PendingCheck{basis, alice_bit};
  used_indices.insert(qi);

  QLOG("[QSDC] Bell check request " << (bell_samples_done + 1)
       << ": qi=" << qi
       << " basis=" << basis
       << " alice_bit=" << alice_bit);

  sendBellCheckRequest(qi, basis);
}

// Message handling logic for the app. Should be broken down in the final version
// Handles the setup response, Bob handling ENT_REQ, Alice handling ENT_RESP elc.
void QSDCApplication::handleMessage(cMessage* msg) {
  // module deletion
  if (dynamic_cast<DeleteThisModule*>(msg)) {
    delete msg;
    deleteModule();
    return;
  }

  if (auto* photon = dynamic_cast<quisp::messages::PhotonicQubit*>(msg)) {
    if (strcmp(photon->getMessage_type(), SAMPLE_PHOTON) == 0) {
      const int qi = (int)photon->par("qubit_index").longValue();
      const int src_addr = (int)photon->par("src_addr").longValue();

      if (photon->isLost()) {
        QLOG("[QSDC] Sample photon lost in channel: qi=" << qi);

        auto* respmsg = new Header(ENT_RESP);
        respmsg->setSrcAddr(my_address);
        respmsg->setDestAddr(src_addr);
        respmsg->setKind(1);

        respmsg->addPar("qubit_index") = qi;
        respmsg->addPar("basis") = "LOST";
        respmsg->addPar("bob_result") = -999;

        send(respmsg, "toRouter");
        delete photon;
        return;
      }

      auto* flying_qubit = const_cast<backends::IQubit*>(photon->getQubitRef());
      if (!flying_qubit) {
        QLOG("[QSDC] Sample photon arrived with null qubit ref: qi=" << qi);
        delete photon;
        return;
      }

      // Bob chooses random basis independently
      const char bob_basis = (dblrand() < 0.5) ? 'Z' : 'X';
      int bob_bit = 0;

      // Proper BB84 intercept-resend model for Eve:
      // Eve measures in a random basis, then resends a fresh state consistent
      // with her basis/result. Instead of physically constructing a new qubit,
      // we compute Bob's outcome from the resend logic directly:
      // - if Bob measures in Eve's basis, Bob gets Eve's bit
      // - otherwise Bob gets a random bit

      // If Eve is not active, Bob measures the original flying qubit normally.
      if (!is_initiator && eve_enabled && dblrand() < eve_intercept_probability) {
        const char eve_basis = (dblrand() < 0.5) ? 'Z' : 'X';
        int eve_bit = 0;

        // Eve measures the original qubit to obtain her result.
        if (eve_basis == 'Z') {
          eve_bit = (eigenToInt(flying_qubit->measureZ()) == +1) ? 0 : 1;
        } else {
          eve_bit = (eigenToInt(flying_qubit->measureX()) == +1) ? 0 : 1;
        }

        // Intercept-resend abstraction:
        // Bob measures the resent qubit, not the already-measured original object.
        if (bob_basis == eve_basis) {
          bob_bit = eve_bit;
        } else {
          bob_bit = (dblrand() < 0.5) ? 0 : 1;
        }

        QLOG("[QSDC] EVE intercepted SAMPLE_PHOTON: qi=" << qi
             << " eve_basis=" << eve_basis
             << " eve_bit=" << eve_bit
             << " bob_basis=" << bob_basis
             << " bob_effective_bit=" << bob_bit
             << " prob=" << eve_intercept_probability);
      } else {
        // No Eve: Bob measures the actual  qubit directly.
        if (bob_basis == 'Z') {
          bob_bit = (eigenToInt(flying_qubit->measureZ()) == +1) ? 0 : 1;
        } else {
          bob_bit = (eigenToInt(flying_qubit->measureX()) == +1) ? 0 : 1;
        }
      }

      QLOG("[QSDC] SAMPLE_PHOTON: Bob measured qi=" << qi
           << " basis=" << bob_basis
           << " bob_bit=" << bob_bit);

      auto* respmsg = new Header(ENT_RESP);
      respmsg->setSrcAddr(my_address);
      respmsg->setDestAddr(src_addr);
      respmsg->setKind(1);

      respmsg->addPar("qubit_index") = qi;
      respmsg->addPar("basis") = std::string(1, bob_basis).c_str();
      respmsg->addPar("bob_result") = bob_bit;

      send(respmsg, "toRouter");
      delete photon;
      return;
    }

    if (strcmp(photon->getMessage_type(), "dense_payload") == 0) {
      const int qi = (int) photon->par("qubit_index").longValue();
      const int src_addr = (int) photon->par("src_addr").longValue();

      if (photon->isLost()) {
        QLOG("[QSDC] Dense photon lost in channel: qi=" << qi);

        auto* done = new Header(DENSE_DONE);
        done->setSrcAddr(my_address);
        done->setDestAddr(src_addr);
        done->setKind(1);
        done->addPar("qubit_index") = qi;
        done->addPar("decoded_bits") = "LOST";

        send(done, "toRouter");
        delete photon;
        return;
      }

      auto* qnic = getLocalEntangledQnic();
      if (!qnic) {
        QLOG("[QSDC] Dense photon arrived, but no qnic found on Bob");
        delete photon;
        return;
      }

      auto* bob_mod = qnic->getSubmodule("statQubit", qi);
      if (!bob_mod) {
        QLOG("[QSDC] Dense photon arrived, but Bob statQubit[" << qi << "] missing");
        delete photon;
        return;
      }

      auto* bob_qubit = check_and_cast<quisp::modules::StationaryQubit*>(bob_mod);

      auto* flying_qubit =
          const_cast<backends::IQubit*>(photon->getQubitRef());

      std::string decoded_bits = decodeDensePair(bob_qubit, flying_qubit);

      QLOG("[QSDC] Dense photon decoded at Bob: qi=" << qi
           << " bits=" << decoded_bits
           << " xErr=" << photon->hasXError()
           << " zErr=" << photon->hasZError());

      auto* done = new Header(DENSE_DONE);
      done->setSrcAddr(my_address);
      done->setDestAddr(src_addr);
      done->setKind(1);
      done->addPar("qubit_index") = qi;
      done->addPar("decoded_bits") = decoded_bits.c_str();

      send(done, "toRouter");

      delete photon;
      return;
    }
  }

  if (strcmp(msg->getName(), BELL_REQ) == 0) {
    auto* hdr = check_and_cast<Header*>(msg);
    const int qi = (int)hdr->par("qubit_index").longValue();
    const std::string basis_str = hdr->par("basis").stringValue();
    const int src_addr = hdr->getSrcAddr();

    auto* qnic = getLocalEntangledQnic();
    if (!qnic) {
      QLOG("[QSDC] BELLCHECK_REQ: no local qnic on Bob");
      delete msg;
      return;
    }

    auto* sq_mod = qnic->getSubmodule("statQubit", qi);
    if (!sq_mod) {
      QLOG("[QSDC] BELLCHECK_REQ: Bob statQubit[" << qi << "] missing");
      delete msg;
      return;
    }

    auto* qubit = check_and_cast<quisp::modules::StationaryQubit*>(sq_mod);
    const char basis = basis_str[0];
    const int bob_bit = measureLocalInBasis(qubit, basis);

    QLOG("[QSDC] BELLCHECK_REQ: Bob measured qi=" << qi
         << " basis=" << basis
         << " bob_bit=" << bob_bit);

    auto* resp = new Header(BELL_RESP);
    resp->setSrcAddr(my_address);
    resp->setDestAddr(src_addr);
    resp->setKind(1);
    resp->addPar("qubit_index") = qi;
    resp->addPar("basis") = std::string(1, basis).c_str();
    resp->addPar("bob_result") = bob_bit;

    send(resp, "toRouter");
    delete msg;
    return;
  }

  if (strcmp(msg->getName(), BELL_RESP) == 0) {
    const int qi = (int)msg->par("qubit_index").longValue();
    const std::string basis_str = msg->par("basis").stringValue();

    auto it = pending_bell_checks.find(qi);
    if (it == pending_bell_checks.end()) {
      QLOG("[QSDC] BELLCHECK_RESP: no pending bell check for qi=" << qi);
      delete msg;
      return;
    }

    const char alice_basis = it->second.basis;
    const int alice_bit = it->second.bit;
    pending_bell_checks.erase(it);

    const char bob_basis = basis_str[0];
    const int bob_bit = (int)msg->par("bob_result").longValue();

    if (alice_basis != bob_basis) {
      QLOG("[QSDC] BELLCHECK_RESP: basis mismatch on qi=" << qi
           << " alice_basis=" << alice_basis
           << " bob_basis=" << bob_basis);
      delete msg;
      scheduleAt(simTime() + sample_interval, new cMessage(SELF_NEXT_BELL_CHECK));
      return;
    }

    const bool pass = expect_anti_correlation ? (alice_bit != bob_bit) : (alice_bit == bob_bit);

    bell_samples_done++;
    if (!pass) bell_errors++;

    const double bell_err_rate =
        (bell_samples_done == 0) ? 0.0 : (double)bell_errors / (double)bell_samples_done;

    QLOG("[QSDC] Bell check result: qi=" << qi
         << " basis=" << alice_basis
         << " alice_bit=" << alice_bit
         << " bob_bit=" << bob_bit
         << " pass=" << (pass ? "YES" : "NO")
         << " samples=" << bell_samples_done
         << " errors=" << bell_errors
         << " error_rate=" << bell_err_rate);

    delete msg;
    scheduleAt(simTime() + sample_interval, new cMessage(SELF_NEXT_BELL_CHECK));
    return;
  }

  // QRSA setup response (from step 3):
  if (auto* resp = dynamic_cast<ConnectionSetupResponse*>(msg)) {
    QLOG("[QSDC] ConnectionSetupResponse received (ruleset_id=" << resp->getRuleSet_id() << ")");

    startQSDCProtocol(resp->getRuleSet_id());
    delete resp;
    return;
  }

  // Handle self messages
  if (msg->isSelfMessage()) {
    if (strcmp(msg->getName(), SELF_START_ONCE) == 0) {
      delete msg;
      startOnce();
      return;
    }
    if (strcmp(msg->getName(), SELF_START_MESSAGE) == 0) {
      delete msg;
      startDenseTransmission();
      return;
    }
    if (strcmp(msg->getName(), SELF_WAIT_FOR_PAIRS) == 0) {
      delete msg;
      pollUntilEnoughPairs();
      return;
    }
    if (strcmp(msg->getName(), SELF_NEXT_SAMPLE) == 0) {
      delete msg;
      doNextSample();
      return;
    }
    if (strcmp(msg->getName(), SELF_NEXT_BELL_CHECK) == 0) {
      delete msg;
      doNextBellCheck();
      return;
    }
  }

  // Step 8: Alice gets DENSE_DONE, which means the dense decoding process is done.
  // reconstruct the text into readable text
  if (strcmp(msg->getName(), DENSE_DONE) == 0) {
    const int qi = (int)msg->par("qubit_index").longValue();
    const std::string bits = msg->par("decoded_bits").stringValue();

    if (bits == "LOST") {
      QLOG("[QSDC] DENSE_DONE: qi=" << qi << " lost in channel");
      bob_decoded_symbols.push_back("??");
    } else {
      bob_decoded_symbols.push_back(bits);
    }

    QLOG("[QSDC] DENSE_DONE: qi=" << qi << " decoded_bits=" << bits);

    if (bob_decoded_symbols.size() == payload_bit_pairs.size()) {
      std::string decoded_bits;
      for (const auto& s : bob_decoded_symbols) decoded_bits += s;

      std::string decoded_text;
      for (size_t i = 0; i + 7 < decoded_bits.size(); i += 8) {
        std::string byte = decoded_bits.substr(i, 8);
        char ch = (char)std::stoi(byte, nullptr, 2);
        decoded_text.push_back(ch);
      }

      QLOG("[QSDC] Dense decoded bits=" << decoded_bits);
      QLOG("[QSDC] Dense decoded text=\"" << decoded_text << "\"");
    }

    delete msg;
    return;
  }

  if (strcmp(msg->getName(), ENT_RESP) == 0) {
    const int qi = (int)msg->par("qubit_index").longValue();
    const std::string basis_str = msg->par("basis").stringValue();

    auto it = pending_checks.find(qi);
    if (it == pending_checks.end()) {
      QLOG("[QSDC] ENT_RESP: no pending check for qi=" << qi);
      delete msg;
      return;
    }

    const char alice_basis = it->second.basis;
    const int alice_bit = it->second.bit;
    pending_checks.erase(it);

    if (basis_str == "LOST") {
      QLOG("[QSDC] Sample result: qi=" << qi << " LOST in channel");

      if (burn_current < burn_count) {
        ++burn_current;
        QLOG("[QSDC] burning sample " << burn_current << "/" << burn_count);
      } else {
        samples_done++;
        errors++;
        const double err_rate = (samples_done == 0) ? 0.0 : (double)errors / (double)samples_done;
        QLOG("[QSDC] Sample result: qi=" << qi
             << " lost=YES"
             << " samples=" << samples_done
             << " errors=" << errors
             << " error_rate=" << err_rate);
      }

      delete msg;
      scheduleAt(simTime() + sample_interval, new cMessage(SELF_NEXT_SAMPLE));
      return;
    }

    const char bob_basis = basis_str[0];
    const int bob_bit = (int)msg->par("bob_result").longValue();

    if (burn_current < burn_count) {
      ++burn_current;
      QLOG("[QSDC] burning sample " << burn_current << "/" << burn_count
           << " qi=" << qi);
      delete msg;
      scheduleAt(simTime() + sample_interval, new cMessage(SELF_NEXT_SAMPLE));
      return;
    }

    if (alice_basis != bob_basis) {
      QLOG("[QSDC] Sample discarded: qi=" << qi
           << " alice_basis=" << alice_basis
           << " bob_basis=" << bob_basis
           << " (basis mismatch)");
      delete msg;
      scheduleAt(simTime() + sample_interval, new cMessage(SELF_NEXT_SAMPLE));
      return;
    }

    const bool pass = (alice_bit == bob_bit);

    samples_done++;
    if (!pass) errors++;

    const double err_rate = (samples_done == 0) ? 0.0 : (double)errors / (double)samples_done;

    QLOG("[QSDC] Sample result: qi=" << qi
         << " alice_basis=" << alice_basis
         << " bob_basis=" << bob_basis
         << " alice_bit=" << alice_bit
         << " bob_bit=" << bob_bit
         << " pass=" << (pass ? "YES" : "NO")
         << " samples=" << samples_done
         << " errors=" << errors
         << " error_rate=" << err_rate);

    delete msg;
    scheduleAt(simTime() + sample_interval, new cMessage(SELF_NEXT_SAMPLE));
    return;
  }

  // Any other QRSA messages
  if (dynamic_cast<ConnectionSetupRequest*>(msg) ||
      dynamic_cast<InternalRuleSetForwarding*>(msg)) {
    logger->logPacket("QSDCApplication::handleMessage", msg);
    send(msg, "toRouter");
    return;
  }

  delete msg;
  QLOG("[QSDC] ERROR: unknown message type received, aborting.");
  error("QSDCApplication: unknown message");
}

// Step 7: if sampling succeeds, then proceed to the actual message sending phase, via dense coding
void QSDCApplication::startDenseTransmission() {
  bob_decoded_symbols.clear();
  payload_indices.clear();
  payload_bit_pairs.clear();

  payload_message_text = std::string(par("payload").stringValue());

  std::string message_bits;
  for (unsigned char ch : payload_message_text) {
    for (int b = 7; b >= 0; --b) {
      message_bits.push_back(((ch >> b) & 1) ? '1' : '0');
    }
  }

  for (size_t i = 0; i < message_bits.size(); i += 2) {
    std::string two = message_bits.substr(i, 2);
    if (two.size() == 1) two.push_back('0');
    payload_bit_pairs.push_back(two);
  }

  std::vector<int> ready;
  countReadyPairsAndCollect(ready);

  for (int qi : ready) {
    if (used_indices.find(qi) == used_indices.end()) {
      payload_indices.push_back(qi);
    }
  }

  if ((int)payload_bit_pairs.size() > (int)payload_indices.size()) {
    QLOG("[QSDC] ERROR: not enough remaining Bell pairs for payload.");
    return;
  }

  QLOG("[QSDC] Dense message text=\"" << payload_message_text << "\"");
  QLOG("[QSDC] Dense message bits=" << message_bits);
  QLOG("[QSDC] Remaining Bell pairs=" << payload_indices.size()
       << " pairs_needed=" << payload_bit_pairs.size());

  auto* qnic = getLocalEntangledQnic();
  if (!qnic) {
    QLOG("[QSDC] ERROR: no local qnic for dense transmission.");
    return;
  }

  for (size_t k = 0; k < payload_bit_pairs.size(); ++k) {
    int qi = payload_indices[k];
    const std::string& bits = payload_bit_pairs[k];

    auto* sq_mod = qnic->getSubmodule("statQubit", qi);
    if (!sq_mod) {
      QLOG("[QSDC] ERROR: dense statQubit[" << qi << "] missing on Alice.");
      continue;
    }

    auto* qubit = check_and_cast<quisp::modules::StationaryQubit*>(sq_mod);

    applyDenseEncoding(qubit, bits);
    used_indices.insert(qi);

    QLOG("[QSDC] Dense encode: qi=" << qi << " bits=" << bits);

    sendDensePhoton(qi, qubit);
    QLOG("[QSDC] Dense send via quantum channel: qi=" << qi);
  }
}

// (part of step 7) Alice uses this to encode her qubit into bell pair, which is sent to Bob
void QSDCApplication::applyDenseEncoding(quisp::modules::StationaryQubit* qubit, const std::string& bits) {
  // Dense coding map for initial phi+:
  // 00 -> I
  // 01 -> X
  // 10 -> Z
  // 11 -> X then Z
  if (bits == "00") {
    return;
  } else if (bits == "01") {
    qubit->gateX();
  } else if (bits == "10") {
    qubit->gateZ();
  } else if (bits == "11") {
    qubit->gateX();
    qubit->gateZ();
  } else {
    throw cRuntimeError("applyDenseEncoding: invalid 2-bit symbol '%s'", bits.c_str());
  }
}

// (part of step 7) Alice uses this to send her dense encoded qubit
void QSDCApplication::sendDensePhoton(int qi, quisp::modules::StationaryQubit* encoded_qubit) {
  auto* photon = new quisp::messages::PhotonicQubit("DENSE_PHOTON");
  photon->setMessage_type("dense_payload");

  // Carry the encoded qubit through the channel
  photon->setQubitRef(encoded_qubit->getBackendQubitRef());

  photon->addPar("src_addr") = my_address;
  photon->addPar("qubit_index") = qi;

  send(photon, "toQuantum");

  QLOG("[QSDC] Dense photon sent through quantum channel: qi=" << qi);
}

// (part of step 7) Bob decodes Alice's qubit
std::string QSDCApplication::decodeDensePair(quisp::modules::StationaryQubit* local_qubit, backends::IQubit* remote_qubit) {
  remote_qubit->gateCNOT(local_qubit->getBackendQubitRef());
  remote_qubit->gateH();

  int first = eigenToInt(remote_qubit->measureZ());
  int second = eigenToInt(local_qubit->measureZ());

  auto bit = [](int x) { return (x == +1) ? '0' : '1'; };

  std::string out;
  out.push_back(bit(first));
  out.push_back(bit(second));
  return out;
}

// Helpers:

// Logic to retrieve bell pairs
omnetpp::cModule* QSDCApplication::getLocalEntangledQnic() {
  auto* qnode = provider.getQNode();
  if (!qnode) return nullptr;

  // Reading bell pairs
  if (is_initiator) {
    if (auto* m = qnode->getSubmodule("qnic_r", 0)) return m;
  } else {
    if (auto* m = qnode->getSubmodule("qnic", 0)) return m;
  }

  // Fallbacks
  if (auto* m = qnode->getSubmodule("qnic_r", 0)) return m;
  if (auto* m = qnode->getSubmodule("qnic", 0)) return m;
  return nullptr;
}

}  // namespace quisp::modules
