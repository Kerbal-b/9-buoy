import QtQuick
import QtQuick.Controls.Basic

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property bool capacityScanRequested: false

    function requestOverviewCapacity() {
        if (capacityScanRequested || !backendObject || backendState.transportMode !== "wifi"
                || !String(backendState.serialStatus || "").startsWith("Connected")
                || !String(backendState.serialTarget || "").startsWith("WIFI:")
                || backendState.sdBusy)
            return
        capacityScanRequested = true
        backendObject.refreshSdFiles()
    }
    onBackendStateChanged: requestOverviewCapacity()
    Component.onCompleted: requestOverviewCapacity()
    onVisibleChanged: {
        if (visible) {
            capacityScanRequested = false
            requestOverviewCapacity()
        }
    }

    function missionTitle() {
        const id = String(backendState.buoyMissionLoadedId || "NONE")
        if (id.toUpperCase() === "NONE" && backendState.radioMissionProgress)
            return "Mission telemetry via Backpack"
        if (id.toUpperCase() === "NONE") return backendState.buoyMissionScanComplete ? "No mission on buoy" : "Mission not checked"
        const mission = (backendState.buoyMissionList || []).find(item => String(item.id || "").toLowerCase() === id.toLowerCase())
        return mission ? String(mission.name || mission.id) : id
    }
    function missionPercentage() {
        const progress = backendState.missionProgress
        const radioProgress = backendState.radioMissionProgress
        if (progress === null && radioProgress && Number.isFinite(Number(radioProgress.percent_complete)))
            return Math.max(0, Math.min(100, Math.round(Number(radioProgress.percent_complete))))
        const count = Number(backendState.buoyMissionExpectedPointCount || backendState.buoyMissionPointCount || 0)
        const loaded = String(backendState.buoyMissionLoadedId || "NONE")
        if (!progress || loaded.toUpperCase() === "NONE" || count <= 0) return null
        const completed = Number(progress.completed_waypoints)
        if (!Number.isFinite(completed)) return null
        return Math.max(0, Math.min(100, Math.round(completed * 100 / count)))
    }
    function missionPointSummary() {
        if (backendState.missionStartBlockReason)
            return "START BLOCKED  |  " + backendState.missionStartBlockReason
        const loaded = String(backendState.buoyMissionLoadedId || "NONE")
        const progress = backendState.missionProgress || backendState.radioMissionProgress
        if (progress) {
            const status = String(progress.status || "MISSION ACTIVE").split("_").join(" ")
            const waypoint = Number(progress.current_waypoint_index)
            const count = Number(progress.waypoint_count)
            const distance = Number(progress.distance_from_home_m)
            const parts = [status]
            if (Number.isFinite(waypoint) && Number.isFinite(count) && count > 0)
                parts.push("WP " + (waypoint + 1) + "/" + count)
            if (Number.isFinite(distance)) parts.push(distance.toFixed(1) + " m from Home")
            if (progress.block_reason) parts.push(String(progress.block_reason))
            return parts.join("  |  ")
        }
        if (loaded.toUpperCase() === "NONE") return "Open Mission Control to upload a mission"
        const count = Number(backendState.buoyMissionExpectedPointCount || backendState.buoyMissionPointCount || 0)
        return count > 0 ? count + " route points stored on buoy" : "Loaded from buoy SD card"
    }

    Rectangle {
        anchors.fill: parent
        radius: 12
        color: "#0b1119"
        border.color: "#395166"
    }

    Flickable {
        id: page
        anchors.fill: parent
        clip: true
        boundsBehavior: Flickable.StopAtBounds
        contentWidth: Math.max(width, inset * 2 + 440 + gap + 230 * 2 + gap)
        contentHeight: Math.max(height, 90 + 3 * rowHeight + 2 * gap + 22)
        readonly property real inset: 22
        readonly property real gap: 14
        readonly property real rowHeight: 160
        readonly property real buoyWidth: 440
        readonly property real gridWidth: contentWidth - inset * 2 - buoyWidth - gap
        readonly property real cardWidth: Math.max(0, (gridWidth - gap) / 2)
        readonly property real cardLeft: inset + buoyWidth + gap
        readonly property real cardTop: 90
        ScrollBar.vertical: ScrollBar { policy: page.contentHeight > page.height ? ScrollBar.AsNeeded : ScrollBar.AlwaysOff }
        ScrollBar.horizontal: ScrollBar { policy: page.contentWidth > page.width ? ScrollBar.AsNeeded : ScrollBar.AlwaysOff }

        Text { x: page.inset; y: 20; text: "Overview"; color: "#e6ecf2"; font.pixelSize: 24; font.bold: true }
        Text { x: page.inset; y: 59; text: "Buoy systems at a glance. Select a tile for details."; color: "#829aad"; font.pixelSize: 13 }

        BuoyMotionTile {
            x: page.inset; y: 90
            width: page.buoyWidth
            height: 2 * page.rowHeight + page.gap
            backendState: root.backendState
        }

        Rectangle {
            x: page.inset; y: 90 + 2 * (page.rowHeight + page.gap)
            width: page.buoyWidth; height: page.rowHeight
            radius: 12
            color: missionMouse.containsMouse ? "#1b3043" : "#111c28"
            border.color: missionMouse.containsMouse ? "#50aaff" : "#395166"
            clip: true
            Column {
                anchors.fill: parent; anchors.margins: 15; spacing: 9
                Text { text: "Mission"; color: "#80bfff"; font.pixelSize: 15; font.bold: true }
                Text { text: root.missionTitle(); width: parent.width; color: "#e6ecf2"; font.pixelSize: 19; font.bold: true; elide: Text.ElideRight }
                Text {
                    text: root.missionPointSummary()
                    width: parent.width
                    color: root.backendState.missionStartBlockReason ? "#ff8c8c" : "#9eb5c7"
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    maximumLineCount: 2
                    elide: Text.ElideRight
                }
                Text {
                    text: root.missionPercentage() === null
                        ? (root.backendState.missionStartBlockReason ? "Mission start blocked" : "Progress not reported by buoy")
                        : root.missionPercentage() + "% complete"
                    color: root.backendState.missionStartBlockReason ? "#ff8c8c" : "#718a9e"
                    font.pixelSize: 11
                    font.bold: Boolean(root.backendState.missionStartBlockReason)
                }
                Rectangle {
                    visible: root.missionPercentage() !== null
                    width: parent.width; height: 10; radius: 5; color: "#20384a"
                    Rectangle { width: parent.width * root.missionPercentage() / 100; height: parent.height; radius: 5; color: "#aa88ec" }
                }
            }
            MouseArea {
                id: missionMouse
                anchors.fill: parent; hoverEnabled: true; cursorShape: Qt.PointingHandCursor
                onClicked: if (root.backendObject) root.backendObject.setDashboardTab("missions")
            }
        }

        OverviewSystemTile {
            x: page.cardLeft; y: page.cardTop
            width: page.cardWidth; height: page.rowHeight
            title: "Communications"; tab: "communications"
            backendObject: root.backendObject; backendState: root.backendState
        }
        OverviewSystemTile {
            x: page.cardLeft + page.cardWidth + page.gap; y: page.cardTop
            width: page.cardWidth; height: page.rowHeight
            title: "Power"; tab: "power"
            backendObject: root.backendObject; backendState: root.backendState
        }
        OverviewSystemTile {
            x: page.cardLeft; y: page.cardTop + page.rowHeight + page.gap
            width: page.cardWidth; height: page.rowHeight
            title: "Instruments"; tab: "instrument_debug"
            backendObject: root.backendObject; backendState: root.backendState
        }
        OverviewSystemTile {
            x: page.cardLeft + page.cardWidth + page.gap; y: page.cardTop + page.rowHeight + page.gap
            width: page.cardWidth; height: page.rowHeight
            title: "Propulsion"; tab: "motors"
            backendObject: root.backendObject; backendState: root.backendState
        }
        OverviewSystemTile {
            x: page.cardLeft; y: page.cardTop + 2 * (page.rowHeight + page.gap)
            width: page.cardWidth; height: page.rowHeight
            title: "Navigation"; tab: "navigation"
            backendObject: root.backendObject; backendState: root.backendState
        }
        OverviewSystemTile {
            x: page.cardLeft + page.cardWidth + page.gap; y: page.cardTop + 2 * (page.rowHeight + page.gap)
            width: page.cardWidth; height: page.rowHeight
            title: "SD Card"; tab: "sd_debug"
            backendObject: root.backendObject; backendState: root.backendState
        }
    }
}
