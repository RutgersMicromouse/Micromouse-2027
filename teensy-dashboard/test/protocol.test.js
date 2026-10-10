const test = require("node:test");
const assert = require("node:assert/strict");
const { parseGraphMessage, parsePidCommand } = require("../protocol");

test("parses supported PID commands as parameter/value pairs", () => {
    assert.deepEqual(parsePidCommand("SET,Kp_dist,6.1"), {
        parameter: "Kp_dist",
        value: 6.1
    });
    assert.deepEqual(parsePidCommand("SET,max_speed,100"), {
        parameter: "max_speed",
        value: 100
    });
});

test("rejects unknown or malformed PID commands", () => {
    assert.equal(parsePidCommand("SET,unknown,1"), null);
    assert.equal(parsePidCommand("SET,Kp_dist,NaN"), null);
    assert.equal(parsePidCommand("SET,Kp_dist,1,extra"), null);
});

test("parses graph reset, node, and cost-bearing edge events", () => {
    assert.deepEqual(parseGraphMessage("GRAPH_RESET"), { type: "reset" });
    assert.deepEqual(parseGraphMessage("GRAPH_NODE,4,-2,3"), {
        type: "node",
        id: 4,
        x: -2,
        y: 3
    });
    assert.deepEqual(parseGraphMessage("GRAPH_EDGE,4,7,2.5"), {
        type: "edge",
        from: 4,
        to: 7,
        cost: 2.5
    });
});

test("accepts legacy edges without costs and rejects invalid graph events", () => {
    assert.deepEqual(parseGraphMessage("GRAPH_EDGE,4,7"), {
        type: "edge",
        from: 4,
        to: 7,
        cost: null
    });
    assert.equal(parseGraphMessage("GRAPH_NODE,one,0,0"), null);
    assert.equal(parseGraphMessage("GRAPH_EDGE,4,7,invalid"), null);
});
