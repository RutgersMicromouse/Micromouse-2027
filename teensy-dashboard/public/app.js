
const status = document.getElementById("status");
const serialLog = document.getElementById("serial-log");
const serialLogClearButton = document.getElementById("serial-log-clear");
const botPosition = document.getElementById("bot-position");
const botAngle = document.getElementById("bot-angle");
const botHeading = document.getElementById("bot-heading");
const botNodeId = document.getElementById("bot-node-id");
const headingNames = ["North", "East", "South", "West"];
const sensorReadouts = {
    left: {
        current: document.getElementById("left-sensor-value"),
        average: document.getElementById("left-sensor-average"),
        values: []
    },
    front: {
        current: document.getElementById("front-sensor-value"),
        average: document.getElementById("front-sensor-average"),
        values: []
    },
    right: {
        current: document.getElementById("right-sensor-value"),
        average: document.getElementById("right-sensor-average"),
        values: []
    }
};

function updateSensorReadout(name, rawValue) {
    const sensor = sensorReadouts[name.toLowerCase()];
    const value = Number(rawValue);

    if (!sensor || !Number.isFinite(value)) {
        return false;
    }

    sensor.values.push(value);
    if (sensor.values.length > 10) {
        sensor.values.shift();
    }

    const average = sensor.values.reduce((sum, sample) => sum + sample, 0) / sensor.values.length;
    sensor.current.textContent = rawValue;
    sensor.average.textContent = String(Math.round(average / 5) * 5);
    return true;
}

const logDisplays = document.querySelectorAll(".log-display");

const socket = new WebSocket(`ws://${location.host}`);

socket.addEventListener("open", () => {
    status.textContent = "Dashboard connected";
    status.style.color = "#4ade80";
});

socket.addEventListener("message", (event) => {
    const graphFields = event.data.split(",");

    if (graphFields[0] === "BOT_STATE" && graphFields.length === 6) {
        const [x, y, angle, heading, nodeId] = graphFields.slice(1).map(Number);
        if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(angle) &&
            Number.isInteger(heading) && Number.isInteger(nodeId)) {
            botPosition.textContent = `(${x}, ${y})`;
            botAngle.textContent = `${angle.toFixed(1)}°`;
            botHeading.textContent = headingNames[heading] || `Unknown (${heading})`;
            botNodeId.textContent = String(nodeId);
            return;
        }
    }

    if (graphFields[0] === "GRAPH_RESET" && graphFields.length === 1) {
        graphNodes.length = 0;
        graphEdges.length = 0;
        showingExampleGraph = false;
        updateNodeSelectors();
        renderGraph();
        graphStatus.textContent = "Bot started a new exploration run.";
        return;
    }

    if (graphFields[0] === "GRAPH_NODE" && graphFields.length === 4) {
        const [id, x, y] = graphFields.slice(1).map(Number);
        if (Number.isInteger(id) && Number.isFinite(x) && Number.isFinite(y)) {
            if (showingExampleGraph) {
                graphNodes.length = 0;
                graphEdges.length = 0;
                showingExampleGraph = false;
            }
            const existingNode = graphNodes.find(node => node.id === id);
            if (existingNode) {
                existingNode.x = x;
                existingNode.y = y;
            } else {
                graphNodes.push({ id, x, y });
            }
            updateNodeSelectors();
            renderGraph();
            graphStatus.textContent = `Received node ${id} at (${x}, ${y}) from the bot.`;
            return;
        }
    }

    if (graphFields[0] === "GRAPH_EDGE" && (graphFields.length === 3 || graphFields.length === 4)) {
        const [from, to] = graphFields.slice(1, 3).map(Number);
        const cost = graphFields.length === 4 ? Number(graphFields[3]) : null;
        if (Number.isInteger(from) && Number.isInteger(to) &&
            (cost === null || Number.isFinite(cost))) {
            const existingEdge = graphEdges.find(edge => edge.from === from && edge.to === to);
            if (existingEdge) {
                existingEdge.cost = cost;
            } else {
                graphEdges.push({ from, to, cost });
            }
            renderGraph();
            graphStatus.textContent = `Received edge ${from} → ${to} with cost ${formatCost(cost)} from the bot.`;
            return;
        }
    }

    if (serialLog.textContent === "Waiting for Teensy data...") {
        serialLog.textContent = "";
    }

    const index = event.data.indexOf('|'); 

    if (index !== -1) {
        const first = event.data.substring(0, index).trim();
        const second = event.data.substring(index + 1).trim();
        if (updateSensorReadout(first, second)) {
            return;
        }
    }

    serialLog.textContent += event.data + "\n";
    serialLog.scrollTop = serialLog.scrollHeight;
});

socket.addEventListener("close", () => {
    status.textContent = "Dashboard disconnected";
    status.style.color = "#f87171";
});

socket.addEventListener("error", () => {
    status.textContent = "Connection error";
});

logDisplays.forEach(logDisplay => {
    logDisplay.querySelector('.clear').addEventListener("click", () => {
        logDisplay.querySelector('.log').textContent = "";
    })
})

const graphNodes = [];
const graphEdges = [];
const graphSvg = document.getElementById("maze-graph");
const graphStatus = document.getElementById("graph-status");
const graphCanvas = graphSvg.parentElement;
const graphZoomLevel = document.getElementById("graph-zoom-level");
const edgeFrom = document.getElementById("edge-from");
const edgeTo = document.getElementById("edge-to");
const addEdgeButton = document.getElementById("add-edge");
const loadExampleButton = document.getElementById("load-example");
const clearGraphButton = document.getElementById("clear-graph");
const svgNamespace = "http://www.w3.org/2000/svg";
const nodeRadius = 36;
const edgeClearance = nodeRadius * 1.5 + 2;
let showingExampleGraph = false;
let graphZoom = 1;
let graphAutoFit = true;
let graphPan = null;

document.getElementById("graph-zoom-in").addEventListener("click", () => {
    zoomGraphAt(1.25, graphCanvas.clientWidth / 2, graphCanvas.clientHeight / 2);
});

document.getElementById("graph-zoom-out").addEventListener("click", () => {
    zoomGraphAt(0.8, graphCanvas.clientWidth / 2, graphCanvas.clientHeight / 2);
});

document.getElementById("graph-zoom-reset").addEventListener("click", () => {
    graphAutoFit = true;
    fitGraphToCanvas();
});

graphCanvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    const bounds = graphCanvas.getBoundingClientRect();
    const pointerX = event.clientX - bounds.left;
    const pointerY = event.clientY - bounds.top;
    zoomGraphAt(event.deltaY < 0 ? 1.1 : 1 / 1.1, pointerX, pointerY);
}, { passive: false });

graphCanvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) {
        return;
    }

    graphAutoFit = false;
    graphPan = {
        pointerId: event.pointerId,
        x: event.clientX,
        y: event.clientY,
        scrollLeft: graphCanvas.scrollLeft,
        scrollTop: graphCanvas.scrollTop
    };
    graphCanvas.classList.add("is-panning");
    graphCanvas.setPointerCapture(event.pointerId);
    event.preventDefault();
});

graphCanvas.addEventListener("pointermove", (event) => {
    if (!graphPan || graphPan.pointerId !== event.pointerId) {
        return;
    }

    graphCanvas.scrollLeft = graphPan.scrollLeft - (event.clientX - graphPan.x);
    graphCanvas.scrollTop = graphPan.scrollTop - (event.clientY - graphPan.y);
});

function finishGraphPan(event) {
    if (!graphPan || graphPan.pointerId !== event.pointerId) {
        return;
    }

    graphPan = null;
    graphCanvas.classList.remove("is-panning");
    if (graphCanvas.hasPointerCapture(event.pointerId)) {
        graphCanvas.releasePointerCapture(event.pointerId);
    }
}

graphCanvas.addEventListener("pointerup", finishGraphPan);
graphCanvas.addEventListener("pointercancel", finishGraphPan);
graphCanvas.addEventListener("lostpointercapture", () => {
    graphPan = null;
    graphCanvas.classList.remove("is-panning");
});

window.addEventListener("resize", () => {
    if (graphAutoFit) {
        fitGraphToCanvas();
    }
});

function loadExampleGraph() {
    graphNodes.splice(0, graphNodes.length,
        { id: 0, x: 0, y: 0 },
        { id: 1, x: 2, y: 0 },
        { id: 2, x: 2, y: 2 },
        { id: 3, x: 4, y: 2 },
        { id: 4, x: 4, y: 4 }
    );
    graphEdges.splice(0, graphEdges.length,
        { from: 0, to: 1, cost: 2 },
        { from: 1, to: 0, cost: 2 },
        { from: 1, to: 2, cost: 3 },
        { from: 2, to: 1, cost: 3 },
        { from: 2, to: 3, cost: 2 },
        { from: 3, to: 2, cost: 2 },
        { from: 3, to: 4, cost: 4 },
        { from: 4, to: 3, cost: 4 },
        { from: 1, to: 4, cost: 6 },
        { from: 4, to: 1, cost: 6 }
    );
    showingExampleGraph = true;
    updateNodeSelectors();
    renderGraph();
    graphStatus.textContent = "Example graph loaded. Bot data will replace it when received.";
}

loadExampleButton.addEventListener("click", loadExampleGraph);
loadExampleGraph();

document.getElementById("node-form").addEventListener("submit", (event) => {
    event.preventDefault();

    const x = Number(document.getElementById("node-x").value);
    const y = Number(document.getElementById("node-y").value);

    if (!Number.isFinite(x) || !Number.isFinite(y)) {
        graphStatus.textContent = "Enter valid X and Y coordinates.";
        return;
    }

    const manualNodeCount = graphNodes.filter(node => typeof node.id === "string").length;
    const id = `web-${manualNodeCount}`;
    graphNodes.push({ id, x, y });
    updateNodeSelectors();
    renderGraph();
    graphStatus.textContent = `Added node ${id} at (${x}, ${y}).`;
});

addEdgeButton.addEventListener("click", () => {
    const from = parseNodeId(edgeFrom.value);
    const to = parseNodeId(edgeTo.value);

    if (from === to) {
        graphStatus.textContent = "Choose two different nodes for an edge.";
        return;
    }

    if (graphEdges.some(edge => edge.from === from && edge.to === to)) {
        graphStatus.textContent = `Edge ${from} → ${to} already exists.`;
        return;
    }

    const cost = Number(document.getElementById("edge-cost").value);
    if (!Number.isFinite(cost) || cost < 0) {
        graphStatus.textContent = "Enter a valid non-negative edge cost.";
        return;
    }

    graphEdges.push({ from, to, cost });
    renderGraph();
    graphStatus.textContent = `Added edge ${from} → ${to} with cost ${formatCost(cost)}.`;
});

function parseNodeId(value) {
    return /^\d+$/.test(value) ? Number(value) : value;
}

clearGraphButton.addEventListener("click", () => {
    graphNodes.length = 0;
    graphEdges.length = 0;
    showingExampleGraph = false;
    updateNodeSelectors();
    renderGraph();
    graphStatus.textContent = "Graph cleared. Add a node to start again.";
});

function updateNodeSelectors() {
    [edgeFrom, edgeTo].forEach(select => {
        select.replaceChildren();
        graphNodes.forEach(node => {
            const option = document.createElement("option");
            option.value = node.id;
            option.textContent = `Node ${node.id} (${node.x}, ${node.y})`;
            select.append(option);
        });
        select.disabled = graphNodes.length < 2;
    });

    addEdgeButton.disabled = graphNodes.length < 2;
    clearGraphButton.disabled = graphNodes.length === 0;
}

function renderGraph() {
    graphSvg.querySelectorAll(".graph-edge, .graph-node, .graph-label, .graph-coordinate, .graph-cost, .graph-axis, .graph-axis-label, .graph-tick, .graph-tick-label, .graph-empty")
        .forEach(element => element.remove());

    if (graphNodes.length === 0) {
        const emptyMessage = document.createElementNS(svgNamespace, "text");
        emptyMessage.classList.add("graph-empty");
        emptyMessage.setAttribute("x", "400");
        emptyMessage.setAttribute("y", "250");
        emptyMessage.textContent = "Your nodes and edges will appear here";
        graphSvg.append(emptyMessage);
        return;
    }

    const positions = getGraphPositions();
    const graphWidth = graphSvg.viewBox.baseVal.width;
    const graphHeight = graphSvg.viewBox.baseVal.height;
    const axisOriginX = 100;
    const axisOriginY = graphHeight - 100;
    const coordinates = graphNodes.flatMap(node => [node.x, node.y]);
    const axisMinimum = Math.min(...coordinates);
    const axisMaximum = Math.max(...coordinates);
    const xAxis = document.createElementNS(svgNamespace, "line");
    xAxis.classList.add("graph-axis");
    xAxis.setAttribute("x1", axisOriginX);
    xAxis.setAttribute("y1", axisOriginY);
    xAxis.setAttribute("x2", graphWidth - 40);
    xAxis.setAttribute("y2", axisOriginY);
    xAxis.setAttribute("marker-end", "url(#edge-arrow)");
    graphSvg.append(xAxis);

    const yAxis = document.createElementNS(svgNamespace, "line");
    yAxis.classList.add("graph-axis");
    yAxis.setAttribute("x1", axisOriginX);
    yAxis.setAttribute("y1", axisOriginY);
    yAxis.setAttribute("x2", axisOriginX);
    yAxis.setAttribute("y2", 40);
    yAxis.setAttribute("marker-end", "url(#edge-arrow)");
    graphSvg.append(yAxis);

    const xAxisLabel = document.createElementNS(svgNamespace, "text");
    xAxisLabel.classList.add("graph-axis-label");
    xAxisLabel.setAttribute("x", graphWidth - 28);
    xAxisLabel.setAttribute("y", axisOriginY + 5);
    xAxisLabel.textContent = "X";
    graphSvg.append(xAxisLabel);

    const yAxisLabel = document.createElementNS(svgNamespace, "text");
    yAxisLabel.classList.add("graph-axis-label");
    yAxisLabel.setAttribute("x", axisOriginX - 5);
    yAxisLabel.setAttribute("y", 28);
    yAxisLabel.textContent = "Y";
    graphSvg.append(yAxisLabel);

    appendAxisTicks("x", axisMinimum, axisMaximum, axisOriginX, axisOriginY);
    appendAxisTicks("y", axisMinimum, axisMaximum, axisOriginX, axisOriginY);

    getUniqueGraphEdges().forEach(edge => {
        const path = document.createElementNS(svgNamespace, "path");
        path.classList.add("graph-edge");
        path.setAttribute("d", getEdgePath(edge, positions));
        path.setAttribute("marker-end", "url(#edge-arrow)");
        if (graphEdges.some(candidate => candidate.from === edge.to && candidate.to === edge.from)) {
            path.setAttribute("marker-start", "url(#edge-arrow)");
        }
        graphSvg.append(path);

        const midpoint = path.getPointAtLength(path.getTotalLength() / 2);
        const costLabel = document.createElementNS(svgNamespace, "text");
        costLabel.classList.add("graph-cost");
        costLabel.setAttribute("x", midpoint.x);
        costLabel.setAttribute("y", midpoint.y);
        costLabel.textContent = formatEdgeCost(edge);
        graphSvg.append(costLabel);
    });

    graphNodes.forEach(node => {
        const position = positions.get(node.id);
        const circle = document.createElementNS(svgNamespace, "circle");
        circle.classList.add("graph-node");
        circle.setAttribute("cx", position.x);
        circle.setAttribute("cy", position.y);
        circle.setAttribute("r", nodeRadius);
        graphSvg.append(circle);

        const label = document.createElementNS(svgNamespace, "text");
        label.classList.add("graph-label");
        label.setAttribute("x", position.x);
        label.setAttribute("y", position.y - 3);
        label.textContent = String(node.id);
        graphSvg.append(label);

        const coordinateLabel = document.createElementNS(svgNamespace, "text");
        coordinateLabel.classList.add("graph-coordinate");
        coordinateLabel.setAttribute("x", position.x);
        coordinateLabel.setAttribute("y", position.y + 14);
        coordinateLabel.textContent = `(${node.x}, ${node.y})`;
        graphSvg.append(coordinateLabel);
    });

    if (graphAutoFit) {
        requestAnimationFrame(fitGraphToCanvas);
    }
}

function appendAxisTicks(axis, minimum, maximum, originX, originY) {
    const cellSpacing = 180;
    const range = maximum - minimum;
    const step = Math.max(1, Math.ceil(range / 8));
    const values = new Set([minimum, maximum]);

    for (let value = Math.ceil(minimum / step) * step; value <= maximum; value += step) {
        values.add(value);
    }

    for (const value of values) {
        const tick = document.createElementNS(svgNamespace, "line");
        tick.classList.add("graph-tick");
        const label = document.createElementNS(svgNamespace, "text");
        label.classList.add("graph-tick-label");

        if (axis === "x") {
            const x = originX + (value - minimum) * cellSpacing;
            tick.setAttribute("x1", x);
            tick.setAttribute("y1", originY - 4);
            tick.setAttribute("x2", x);
            tick.setAttribute("y2", originY + 4);
            label.setAttribute("x", x);
            label.setAttribute("y", originY + 18);
        } else {
            const y = originY - (value - minimum) * cellSpacing;
            tick.setAttribute("x1", originX - 4);
            tick.setAttribute("y1", y);
            tick.setAttribute("x2", originX + 4);
            tick.setAttribute("y2", y);
            label.setAttribute("x", originX - 8);
            label.setAttribute("y", y + 4);
            label.setAttribute("text-anchor", "end");
        }

        label.textContent = String(Number(value.toFixed(2)));
        graphSvg.append(tick, label);
    }
}

function getGraphPositions() {
    const xValues = graphNodes.map(node => node.x);
    const yValues = graphNodes.map(node => node.y);
    const minX = Math.min(...xValues);
    const maxX = Math.max(...xValues);
    const minY = Math.min(...yValues);
    const maxY = Math.max(...yValues);
    const axisMinimum = Math.min(minX, minY);
    const axisMaximum = Math.max(maxX, maxY);
    const padding = 100;
    const cellSpacing = 180;
    const minimumNodeSpacing = 100;
    const positions = new Map(graphNodes.map(node => [node.id, {
        x: (node.x - axisMinimum) * cellSpacing,
        y: (axisMaximum - node.y) * cellSpacing
    }]));

    for (let iteration = 0; iteration < 80; iteration++) {
        let movedNodes = false;

        for (let index = 0; index < graphNodes.length; index++) {
            for (let otherIndex = index + 1; otherIndex < graphNodes.length; otherIndex++) {
                const firstPosition = positions.get(graphNodes[index].id);
                const secondPosition = positions.get(graphNodes[otherIndex].id);
                let deltaX = secondPosition.x - firstPosition.x;
                let deltaY = secondPosition.y - firstPosition.y;
                let distance = Math.hypot(deltaX, deltaY);

                if (distance >= minimumNodeSpacing) {
                    continue;
                }

                movedNodes = true;
                if (distance === 0) {
                    const angle = (index * 2.399 + otherIndex) % (Math.PI * 2);
                    deltaX = Math.cos(angle);
                    deltaY = Math.sin(angle);
                    distance = 1;
                }

                const separation = (minimumNodeSpacing - distance) / 2;
                const directionX = deltaX / distance;
                const directionY = deltaY / distance;
                firstPosition.x -= directionX * separation;
                firstPosition.y -= directionY * separation;
                secondPosition.x += directionX * separation;
                secondPosition.y += directionY * separation;
            }
        }

        if (!movedNodes) {
            break;
        }
    }

    resolveEdgeCrossings(positions, minimumNodeSpacing);

    const adjustedPositions = Array.from(positions.values());
    const adjustedMinX = Math.min(...adjustedPositions.map(position => position.x));
    const adjustedMaxX = Math.max(...adjustedPositions.map(position => position.x));
    const adjustedMinY = Math.min(...adjustedPositions.map(position => position.y));
    const adjustedMaxY = Math.max(...adjustedPositions.map(position => position.y));
    const squareSize = Math.max(
        800,
        (axisMaximum - axisMinimum) * cellSpacing + padding * 2,
        adjustedMaxX - adjustedMinX + padding * 2,
        adjustedMaxY - adjustedMinY + padding * 2
    );
    const width = squareSize;
    const height = squareSize;

    graphSvg.setAttribute("viewBox", `0 0 ${width} ${height}`);
    graphSvg.setAttribute("width", width);
    graphSvg.setAttribute("height", height);

    positions.forEach(position => {
        position.x = position.x - adjustedMinX + padding;
        position.y = position.y - adjustedMinY + padding;
    });

    return positions;
}

function zoomGraphAt(factor, pointerX, pointerY) {
    const previousZoom = graphZoom;
    const minimumZoom = getFitZoom();
    const maximumZoom = Math.max(4, minimumZoom);
    const nextZoom = Math.min(maximumZoom, Math.max(minimumZoom, graphZoom * factor));
    if (nextZoom === previousZoom) {
        return;
    }

    graphAutoFit = false;
    const graphPointX = (graphCanvas.scrollLeft + pointerX) / previousZoom;
    const graphPointY = (graphCanvas.scrollTop + pointerY) / previousZoom;
    graphZoom = nextZoom;
    applyGraphZoom();
    graphCanvas.scrollLeft = Math.max(0, graphPointX * graphZoom - pointerX);
    graphCanvas.scrollTop = Math.max(0, graphPointY * graphZoom - pointerY);
}

function applyGraphZoom() {
    const viewBox = graphSvg.viewBox.baseVal;
    if (!viewBox.width || !viewBox.height) {
        return;
    }

    graphSvg.setAttribute("width", Math.round(viewBox.width * graphZoom));
    graphSvg.setAttribute("height", Math.round(viewBox.height * graphZoom));
    graphZoomLevel.textContent = `${Math.round(graphZoom * 100)}%`;
}

function getFitZoom() {
    const viewBox = graphSvg.viewBox.baseVal;
    if (!viewBox.width || !viewBox.height || !graphCanvas.clientWidth || !graphCanvas.clientHeight) {
        return graphZoom;
    }

    return Math.min(graphCanvas.clientWidth / viewBox.width, graphCanvas.clientHeight / viewBox.height);
}

function fitGraphToCanvas() {
    if (!graphSvg.viewBox.baseVal.width || !graphCanvas.clientWidth || !graphCanvas.clientHeight) {
        return;
    }

    graphZoom = getFitZoom();
    applyGraphZoom();
    graphCanvas.scrollLeft = 0;
    graphCanvas.scrollTop = 0;
}

function getEdgePath(edge, positions) {
    const start = positions.get(edge.from);
    const end = positions.get(edge.to);
    const midpoint = { x: (start.x + end.x) / 2, y: (start.y + end.y) / 2 };
    const deltaX = end.x - start.x;
    const deltaY = end.y - start.y;
    const length = Math.hypot(deltaX, deltaY) || 1;
    const perpendicular = { x: -deltaY / length, y: deltaX / length };
    const maxOffset = 240;
    const offsets = [0];

    for (let offset = edgeClearance; offset <= maxOffset; offset += edgeClearance) {
        offsets.push(offset, -offset);
    }

    for (const offset of offsets) {
        const control = {
            x: midpoint.x + perpendicular.x * offset,
            y: midpoint.y + perpendicular.y * offset
        };
        const start = getCircleExitPoint(positions.get(edge.from), control, edgeClearance);
        const end = getCircleExitPoint(positions.get(edge.to), control, edgeClearance);

        if (isEdgePathClear(start, control, end, positions)) {
            return `M ${start.x} ${start.y} Q ${control.x} ${control.y} ${end.x} ${end.y}`;
        }
    }

    const fallbackStart = getCircleExitPoint(positions.get(edge.from), midpoint, edgeClearance);
    const fallbackEnd = getCircleExitPoint(positions.get(edge.to), midpoint, edgeClearance);
    return `M ${fallbackStart.x} ${fallbackStart.y} L ${fallbackEnd.x} ${fallbackEnd.y}`;
}

function getCircleExitPoint(center, toward, clearance) {
    const deltaX = toward.x - center.x;
    const deltaY = toward.y - center.y;
    const length = Math.hypot(deltaX, deltaY) || 1;

    return {
        x: center.x + deltaX / length * clearance,
        y: center.y + deltaY / length * clearance
    };
}

function isEdgePathClear(start, control, end, positions) {
    const sampleCount = 48;

    for (let sample = 1; sample < sampleCount; sample++) {
        const progress = sample / sampleCount;
        const inverseProgress = 1 - progress;
        const point = {
            x: inverseProgress ** 2 * start.x + 2 * inverseProgress * progress * control.x + progress ** 2 * end.x,
            y: inverseProgress ** 2 * start.y + 2 * inverseProgress * progress * control.y + progress ** 2 * end.y
        };

        for (const center of positions.values()) {
            if (Math.hypot(point.x - center.x, point.y - center.y) < edgeClearance) {
                return false;
            }
        }
    }

    return true;
}

function resolveEdgeCrossings(positions, minimumNodeSpacing) {
    const edges = getUniqueGraphEdges();
    const directions = [
        { x: 1, y: 0 },
        { x: -1, y: 0 },
        { x: 0, y: 1 },
        { x: 0, y: -1 },
        { x: 1, y: 1 },
        { x: 1, y: -1 },
        { x: -1, y: 1 },
        { x: -1, y: -1 }
    ];

    for (let iteration = 0; iteration < edges.length * 2; iteration++) {
        const currentCrossings = countEdgeCrossings(edges, positions);
        if (currentCrossings === 0) {
            return;
        }

        const crossing = findCrossingEdges(edges, positions);
        if (!crossing) {
            return;
        }

        const nodeIds = [crossing.first.from, crossing.first.to, crossing.second.from, crossing.second.to];
        let bestMove = null;
        let bestCrossings = currentCrossings;

        for (const nodeId of nodeIds) {
            const position = positions.get(nodeId);

            for (const direction of directions) {
                const directionLength = Math.hypot(direction.x, direction.y);
                for (const distance of [minimumNodeSpacing, minimumNodeSpacing * 2]) {
                    const candidate = {
                        x: position.x + direction.x / directionLength * distance,
                        y: position.y + direction.y / directionLength * distance
                    };

                    if (!isNodePositionClear(nodeId, candidate, positions, minimumNodeSpacing)) {
                        continue;
                    }

                    const original = { x: position.x, y: position.y };
                    positions.set(nodeId, candidate);
                    const candidateCrossings = countEdgeCrossings(edges, positions);
                    positions.set(nodeId, original);

                    if (candidateCrossings < bestCrossings) {
                        bestCrossings = candidateCrossings;
                        bestMove = { nodeId, position: candidate };
                    }
                }
            }
        }

        if (!bestMove) {
            return;
        }

        positions.set(bestMove.nodeId, bestMove.position);
    }
}

function getUniqueGraphEdges() {
    const seen = new Set();

    return graphEdges.filter(edge => {
        const endpoints = [String(edge.from), String(edge.to)].sort();
        const key = `${endpoints[0]}:${endpoints[1]}`;
        if (seen.has(key)) {
            return false;
        }
        seen.add(key);
        return true;
    });
}

function formatCost(cost) {
    if (cost === null || cost === undefined || !Number.isFinite(Number(cost))) {
        return "?";
    }

    return String(Number(Number(cost).toFixed(2)));
}

function formatEdgeCost(edge) {
    const reverseEdge = graphEdges.find(candidate => candidate.from === edge.to && candidate.to === edge.from);
    if (reverseEdge && Number(reverseEdge.cost) !== Number(edge.cost)) {
        return `${formatCost(edge.cost)} / ${formatCost(reverseEdge.cost)}`;
    }

    return formatCost(edge.cost);
}

function countEdgeCrossings(edges, positions) {
    let crossings = 0;

    for (let index = 0; index < edges.length; index++) {
        for (let otherIndex = index + 1; otherIndex < edges.length; otherIndex++) {
            const first = edges[index];
            const second = edges[otherIndex];
            if (first.from === second.from || first.from === second.to ||
                first.to === second.from || first.to === second.to) {
                continue;
            }

            if (segmentsIntersect(
                positions.get(first.from),
                positions.get(first.to),
                positions.get(second.from),
                positions.get(second.to)
            )) {
                crossings++;
            }
        }
    }

    return crossings;
}

function findCrossingEdges(edges, positions) {
    for (let index = 0; index < edges.length; index++) {
        for (let otherIndex = index + 1; otherIndex < edges.length; otherIndex++) {
            const first = edges[index];
            const second = edges[otherIndex];
            if (first.from === second.from || first.from === second.to ||
                first.to === second.from || first.to === second.to) {
                continue;
            }

            if (segmentsIntersect(
                positions.get(first.from),
                positions.get(first.to),
                positions.get(second.from),
                positions.get(second.to)
            )) {
                return { first, second };
            }
        }
    }

    return null;
}

function segmentsIntersect(firstStart, firstEnd, secondStart, secondEnd) {
    const orientation = (start, end, point) =>
        (end.x - start.x) * (point.y - start.y) - (end.y - start.y) * (point.x - start.x);
    const onSegment = (start, point, end) =>
        point.x >= Math.min(start.x, end.x) && point.x <= Math.max(start.x, end.x) &&
        point.y >= Math.min(start.y, end.y) && point.y <= Math.max(start.y, end.y);
    const epsilon = 0.001;
    const firstSide = orientation(firstStart, firstEnd, secondStart);
    const secondSide = orientation(firstStart, firstEnd, secondEnd);
    const thirdSide = orientation(secondStart, secondEnd, firstStart);
    const fourthSide = orientation(secondStart, secondEnd, firstEnd);

    if ((firstSide > epsilon && secondSide < -epsilon || firstSide < -epsilon && secondSide > epsilon) &&
        (thirdSide > epsilon && fourthSide < -epsilon || thirdSide < -epsilon && fourthSide > epsilon)) {
        return true;
    }

    return Math.abs(firstSide) <= epsilon && onSegment(firstStart, secondStart, firstEnd) ||
        Math.abs(secondSide) <= epsilon && onSegment(firstStart, secondEnd, firstEnd) ||
        Math.abs(thirdSide) <= epsilon && onSegment(secondStart, firstStart, secondEnd) ||
        Math.abs(fourthSide) <= epsilon && onSegment(secondStart, firstEnd, secondEnd);
}

function isNodePositionClear(nodeId, candidate, positions, minimumNodeSpacing) {
    for (const [otherId, otherPosition] of positions) {
        if (otherId === nodeId) {
            continue;
        }

        if (Math.hypot(candidate.x - otherPosition.x, candidate.y - otherPosition.y) < minimumNodeSpacing) {
            return false;
        }
    }

    return true;
}

function setPID(parameter, inputId) {
  const input = document.getElementById(inputId);
  const value = Number(input.value);
  const status = document.getElementById("pid-status");

  if (input.value === "" || !Number.isFinite(value)) {
    status.textContent = "Enter a valid number.";
    return;
  }

  if (!socket || socket.readyState !== WebSocket.OPEN) {
    status.textContent = "Not connected to the dashboard server.";
    return;
  }

  socket.send(`SET,${parameter},${value}`);
  status.textContent = `Sent ${parameter} = ${value}`;
}
