package frc.robot.subsystems;

import com.sun.net.httpserver.HttpServer;
import com.sun.net.httpserver.HttpExchange;

import edu.wpi.first.wpilibj.RobotController;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class RobovikesNet {

    private static final int PORT = 4444;

    // WPILib deploy directory
    private static final Path WEB_ROOT = Paths.get("/home/lvuser/deploy");

    private HttpServer server;

    public RobovikesNet() {
        try {
            server = HttpServer.create(new InetSocketAddress(PORT), 0);

            server.createContext("/voltage", this::handleVoltage);
            server.createContext("/", this::handleFiles);

            server.setExecutor(null);
            server.start();

            System.out.println("[RobovikesNet] Web server started on port " + PORT);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /* ---------------- Voltage Endpoint ---------------- */

    private void handleVoltage(HttpExchange exchange) throws IOException {
        double voltage = RobotController.getBatteryVoltage();
        String response = "{ \"voltage\": " + voltage + " }";

        exchange.getResponseHeaders().add("Content-Type", "application/json");
        exchange.sendResponseHeaders(200, response.length());

        try (OutputStream os = exchange.getResponseBody()) {
            os.write(response.getBytes());
        }
    }

    /* ---------------- Static File Handler ---------------- */

    private void handleFiles(HttpExchange exchange) throws IOException {
        String uriPath = exchange.getRequestURI().getPath();

        if (uriPath.equals("/")) {
            uriPath = "/index.html";
        }

        Path filePath = WEB_ROOT.resolve("." + uriPath).normalize();

        // Prevent path traversal & missing files
        if (!filePath.startsWith(WEB_ROOT) || !Files.exists(filePath)) {
            exchange.sendResponseHeaders(404, -1);
            return;
        }

        exchange.getResponseHeaders().add(
                "Content-Type", getContentType(filePath));

        byte[] bytes = Files.readAllBytes(filePath);
        exchange.sendResponseHeaders(200, bytes.length);

        try (OutputStream os = exchange.getResponseBody()) {
            os.write(bytes);
        }
    }

    private String getContentType(Path path) {
        String name = path.toString();
        if (name.endsWith(".html")) return "text/html";
        if (name.endsWith(".css")) return "text/css";
        if (name.endsWith(".js")) return "application/javascript";
        return "application/octet-stream";
    }
}
