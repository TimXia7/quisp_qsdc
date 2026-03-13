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

// Sampling protocol message names
static const char* ENT_REQ = "ENTCHECK_REQ";
static const char* ENT_RESP = "ENTCHECK_RESP";

// Dense coding message names
static const char* SELF_START_MESSAGE = "START_MESSAGE";
static const char* DENSE_DONE = "DENSE_DONE";

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

  start_delay = par("start_delay");
  poll_interval = par("poll_interval");
  sample_interval = par("sample_interval");
  burn_count = par("burn_count").intValue();
  burn_current = 0;

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

  active_ruleset_id = ruleset_id;

  // Reset sampling stats
  samples_done = 0;
  errors = 0;
  sampling_started = false;
  used_indices.clear();
  pending_checks.clear();

  QLOG("[QSDC] Starting sampling after delay=" << start_delay
       << ", waiting for >= " << min_pairs_to_start << " ready pairs.");

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

// Step 5: Sampling fully begins; it loops to keep sampling until all samples are finished (samples_done >= sample_target)
// When sampling is allowed to start, it picks an available bellpair and measure's Alice's qubit
// in a specific basis, and which pair Bob should measure in response.
// After Bob does this, Alice continues to the next
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
      QLOG("[QSDC] Channel accepted; starting dense-coded message transmission.");
      scheduleAt(simTime() + sample_interval, new cMessage(SELF_START_MESSAGE));
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

  // Random basis selection (X or Z)
  const char basis = (dblrand() < 0.5) ? 'Z' : 'X';

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

  // Measure Alice half
  int alice_res = 0;
  if (basis == 'Z') alice_res = eigenToInt(qubit->measureZ());
  else alice_res = eigenToInt(qubit->measureX());

  pending_checks[qi] = PendingCheck{basis, alice_res};
  used_indices.insert(qi);

  QLOG("[QSDC] Sample " << (samples_done + 1)
       << ": qi=" << qi << " basis=" << basis
       << " alice=" << alice_res
       << " (sending ENTCHECK_REQ to Bob)");

  // Send request to Bob using Header so Router can deliver kind==1 to the app
  const int bob_addr = par("bob_addr").intValue();
  auto* req = new Header(ENT_REQ);
  req->setSrcAddr(my_address);
  req->setDestAddr(bob_addr);
  req->setKind(1);

  req->addPar("src_addr") = my_address;
  req->addPar("qubit_index") = qi;
  req->addPar("basis") = std::string(1, basis).c_str();

  send(req, "toRouter");
  // Next sample is scheduled after ENT_RESP arrives.
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

  // (part of step 6) Bob's messages side: receives ENTCHECK_REQ, the samples from Alice
  if (strcmp(msg->getName(), ENT_REQ) == 0) {
    const int src_addr = (int)msg->par("src_addr").longValue();
    const int qi = (int)msg->par("qubit_index").longValue();
    const char basis = msg->par("basis").stringValue()[0];

    auto* qnic = getLocalEntangledQnic();  // on Bob this picks qnic[0]
    if (!qnic) {
      QLOG("[QSDC] ENT_REQ: no qnic found on this node");
      delete msg;
      return;
    }

    auto* sq_mod = qnic->getSubmodule("statQubit", qi);
    if (!sq_mod) {
      QLOG("[QSDC] ENT_REQ: statQubit[" << qi << "] not found");
      delete msg;
      return;
    }

    auto* qubit = check_and_cast<quisp::modules::StationaryQubit*>(sq_mod);

    if (qubit->isLocked()) {
      QLOG("[QSDC] ENT_REQ: qubit[" << qi << "] locked; skipping");
      delete msg;
      return;
    }

    int bob_res = 0;
    if (basis == 'Z') bob_res = eigenToInt(qubit->measureZ());
    else if (basis == 'X') bob_res = eigenToInt(qubit->measureX());
    else {
      QLOG("[QSDC] ENT_REQ: unsupported basis " << basis);
      delete msg;
      return;
    }

    QLOG("[QSDC] ENT_REQ: measured qi=" << qi
         << " basis=" << basis
         << " bob=" << bob_res);

    auto* respmsg = new Header(ENT_RESP);
    respmsg->setSrcAddr(my_address);
    respmsg->setDestAddr(src_addr);
    respmsg->setKind(1);

    respmsg->addPar("qubit_index") = qi;
    respmsg->addPar("basis") = std::string(1, basis).c_str();
    respmsg->addPar("bob_result") = bob_res;

    send(respmsg, "toRouter");
    delete msg;
    return;
  }

  // (part of step 6) Alice's messages side: receives ENT_RESP, the sample response from Bob
  if (strcmp(msg->getName(), ENT_RESP) == 0) {

    if (burn_current < burn_count) {
      ++burn_current;
      QLOG("[QSDC] burning qubit " << burn_current << "/" << burn_count);
      delete msg;
      scheduleAt(simTime() + sample_interval, new cMessage(SELF_NEXT_SAMPLE));
      return;
    }

    const int qi = (int)msg->par("qubit_index").longValue();
    const char basis = msg->par("basis").stringValue()[0];
    const int bob_res = (int)msg->par("bob_result").longValue();

    auto it = pending_checks.find(qi);
    if (it == pending_checks.end()) {
      QLOG("[QSDC] ENT_RESP: no pending check for qi=" << qi);
      delete msg;
      return;
    }

    const int alice_res = it->second.alice_result;
    pending_checks.erase(it);

    // Determine pass/fail based on expected relation
    bool pass = false;
    if (expect_anti_correlation) pass = (alice_res == -bob_res);
    else pass = (alice_res == bob_res);

    samples_done++;
    if (!pass) errors++;

    const double err_rate = (samples_done == 0) ? 0.0 : (double)errors / (double)samples_done;

    QLOG("[QSDC] Sample result: qi=" << qi
         << " basis=" << basis
         << " alice=" << alice_res
         << " bob=" << bob_res
         << " pass=" << (pass ? "YES" : "NO")
         << " samples=" << samples_done
         << " errors=" << errors
         << " error_rate=" << err_rate);

    // Schedule next sample
    scheduleAt(simTime() + sample_interval, new cMessage(SELF_NEXT_SAMPLE));

    delete msg;
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
