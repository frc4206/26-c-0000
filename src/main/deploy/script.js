async function updateVoltage() {
    try {
        const response = await fetch("/voltage");
        const data = await response.json();
        document.getElementById("voltage").textContent =
            data.voltage.toFixed(2) + " V";
    } catch (e) {
        document.getElementById("voltage").textContent = "ERR";
    }
}

setInterval(updateVoltage, 200);
updateVoltage();
