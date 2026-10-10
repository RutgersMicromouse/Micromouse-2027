const pidParameters = new Set([
    "Kp_dist",
    "Ki_dist",
    "Kd_dist",
    "Kp_angle",
    "Ki_angle",
    "Kd_angle",
    "Kp_lat",
    "Kd_lat",
    "Kp_wall",
    "Kd_wall",
    "max_speed"
]);

function parsePidCommand(command) {
    const match = command.trim().match(/^SET,([A-Za-z_]+),(-?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)$/);
    if (!match || !pidParameters.has(match[1])) {
        return null;
    }

    const value = Number(match[2]);
    if (!Number.isFinite(value)) {
        return null;
    }

    return { parameter: match[1], value };
}

function parseGraphMessage(message) {
    const fields = message.trim().split(",");

    if (fields[0] === "GRAPH_RESET" && fields.length === 1) {
        return { type: "reset" };
    }

    if (fields[0] === "GRAPH_NODE" && fields.length === 4) {
        const [id, x, y] = fields.slice(1).map(Number);
        if (Number.isInteger(id) && Number.isFinite(x) && Number.isFinite(y)) {
            return { type: "node", id, x, y };
        }
        return null;
    }

    if (fields[0] === "GRAPH_EDGE" && (fields.length === 3 || fields.length === 4)) {
        const [from, to] = fields.slice(1, 3).map(Number);
        const cost = fields.length === 4 ? Number(fields[3]) : null;
        if (Number.isInteger(from) && Number.isInteger(to) &&
            (cost === null || Number.isFinite(cost))) {
            return { type: "edge", from, to, cost };
        }
    }

    return null;
}

module.exports = { parseGraphMessage, parsePidCommand };
