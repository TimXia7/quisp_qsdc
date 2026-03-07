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

  // separate namespace to isolate logging
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

// const chars that determine simulation settings
static const char* SELF_START_ONCE = "START_ONCE";
static const char* SELF_WAIT_FOR_PAIRS = "WAIT_FOR_PAIRS";
static const char* SELF_NEXT_SAMPLE = "NEXT_SAMPLE";

// Sampling protocol message names
static const char* ENT_REQ = "ENTCHECK_REQ";
static const char* ENT_RESP = "ENTCHECK_RESP";

// Convert eigenvalue result to +/-1
// simplifies vector calculations of qubits
static inline int eigenToInt(quisp::types::EigenvalueResult r) {
  return (r == quisp::types::EigenvalueResult::PLUS_ONE) ? +1 : -1;
}

// runs on program startup:
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

  if (is_initiator) {
    scheduleAt(simTime(), new cMessage(SELF_START_ONCE));
  }
}

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

// Calculate the amount of available bell pairs
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

// keep polling after the initial wait until there are enough pairs to initiate the process
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
  // QRSA setup response:
  if (auto* resp = dynamic_cast<ConnectionSetupResponse*>(msg)) {
    QLOG("[QSDC] ConnectionSetupResponse received (ruleset_id=" << resp->getRuleSet_id() << ")");

    startQSDCProtocol(resp->getRuleSet_id());
    delete resp;
    return;
  }

  // module deletion
  if (dynamic_cast<DeleteThisModule*>(msg)) {
    delete msg;
    deleteModule();
    return;
  }

  // Handle self messages
  if (msg->isSelfMessage()) {
    if (strcmp(msg->getName(), SELF_START_ONCE) == 0) {
      delete msg;
      startOnce();
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

  // Bob's messages side: receives ENTCHECK_REQ
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

  // Alice's messages : receives ENTCHECK_RESP
  if (strcmp(msg->getName(), ENT_RESP) == 0) {
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

// Startup the process
// This may change if I choose to have the simulation run constantly, like a real network
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

// preliminary settings for the protocol starting.
// Once the delay is over, the polling starts to check if there are enough pairs to start QSDC
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

}  // namespace quisp::modules
