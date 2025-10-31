// Programa completo para dos esferas que se atraen y dibujan conexiones ondulantes tipo "fusión"

#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iostream>

const int SCREEN_W = 900;
const int SCREEN_H = 600;
const float SPHERE_RADIUS = 140.0f;
const int NUM_POINTS = 120;
const int NUM_AURORA_STRANDS = 50;
const float CONNECTION_DISTANCE = 90.0f;
const float ATTRACTION_DISTANCE = 400.0f;
const float ATTRACTION_STRENGTH = 0.015f;

struct AuroraStrand {
    float baseAngle;
    float phase;
    float speed;
    float amplitude;
    float length;
    float colorShift;
    float noiseOffset;
};

struct Point3D {
    float x, y, z;
    float originalX, originalY, originalZ;
    float phase;
    float speed;
};

struct Sphere {
    float centerX, centerY;
    float velX, velY;
    float rotationY, rotationX;
    float time;
    float breathe;
    float stretchX, stretchY;
    float stretchVelX, stretchVelY;
    float formationProgress;
    bool isForming;
    float auroraPhase;
    float auroraIntensity;
    int r, g, b;
    std::vector<Point3D> points;
    std::vector<AuroraStrand> auroraStrands;
};

static inline int clampInt(int v, int a = 0, int b = 255) {
    return (v < a) ? a : (v > b) ? b : v;
}

float simpleNoise(float x, float y) {
    return sinf(x * 1.3f + y * 0.7f) * 0.5f +
           sinf(x * 2.1f - y * 1.4f) * 0.25f +
           sinf(x * 0.8f + y * 2.3f) * 0.125f;
}

void initSphere(Sphere& sphere, int colorIndex) {
    sphere.centerX = SCREEN_W / 2.0f;
    sphere.centerY = SCREEN_H / 2.0f;
    sphere.velX = 0;
    sphere.velY = 0;
    sphere.rotationY = 0;
    sphere.rotationX = 0;
    sphere.time = 0;
    sphere.breathe = 1.0f;
    sphere.stretchX = 1.0f;
    sphere.stretchY = 1.0f;
    sphere.stretchVelX = 0;
    sphere.stretchVelY = 0;
    sphere.formationProgress = 0.0f;
    sphere.isForming = true;
    sphere.auroraPhase = 0.0f;
    sphere.auroraIntensity = 0.0f;

    if (colorIndex == 0) {
        sphere.r = 50; sphere.g = 240; sphere.b = 130;
    } else {
        sphere.r = 90; sphere.g = 160; sphere.b = 240;
    }

    sphere.auroraStrands.clear();
    for (int i = 0; i < NUM_AURORA_STRANDS; i++) {
        AuroraStrand strand;
        strand.baseAngle = (float)i / NUM_AURORA_STRANDS * 2.0f * ALLEGRO_PI;
        strand.phase = (float)rand() / RAND_MAX * 2.0f * ALLEGRO_PI;
        strand.speed = 0.4f + (float)rand() / RAND_MAX * 0.6f;
        strand.amplitude = 35.0f + (float)rand() / RAND_MAX * 45.0f;
        strand.length = 120.0f + (float)rand() / RAND_MAX * 80.0f;
        strand.colorShift = (float)rand() / RAND_MAX;
        strand.noiseOffset = (float)rand() / RAND_MAX * 100.0f;
        sphere.auroraStrands.push_back(strand);
    }

    sphere.points.clear();
    const float goldenAngle = ALLEGRO_PI * (3.0f - sqrtf(5.0f));
    for (int i = 0; i < NUM_POINTS; i++) {
        Point3D p;
        float y = 1.0f - (i / (float)(NUM_POINTS - 1)) * 2.0f;
        float radius = sqrtf(1.0f - y * y);
        float theta = goldenAngle * i;

        p.originalX = cosf(theta) * radius * SPHERE_RADIUS;
        p.originalY = y * SPHERE_RADIUS;
        p.originalZ = sinf(theta) * radius * SPHERE_RADIUS;

        float randomDist = 300.0f + (float)rand() / RAND_MAX * 200.0f;
        float randomAngle = (float)rand() / RAND_MAX * 2.0f * ALLEGRO_PI;
        float randomHeight = ((float)rand() / RAND_MAX - 0.5f) * 400.0f;

        p.x = cosf(randomAngle) * randomDist;
        p.y = randomHeight;
        p.z = sinf(randomAngle) * randomDist;

        p.phase = (float)rand() / RAND_MAX * 2.0f * ALLEGRO_PI;
        p.speed = 0.3f + (float)rand() / RAND_MAX * 0.5f;

        sphere.points.push_back(p);
    }
}

Point3D rotatePoint(Point3D p, float rotX, float rotY) {
    Point3D result;
    float x1 = p.x * cosf(rotY) - p.z * sinf(rotY);
    float z1 = p.x * sinf(rotY) + p.z * cosf(rotY);
    result.x = x1;
    result.y = p.y * cosf(rotX) - z1 * sinf(rotX);
    result.z = p.y * sinf(rotX) + z1 * cosf(rotX);
    return result;
}

struct DrawPoint {
    float z;
    Point3D rotated;
    float screenX;
    float screenY;
    bool operator<(const DrawPoint& other) const {
        return z < other.z;
    }
};

void updateSphere(Sphere& sphere, int prevX, int prevY, int currX, int currY, float dt, const std::vector<std::pair<float, float>>& others) {
    float deltaX = (float)(currX - prevX);
    float deltaY = (float)(currY - prevY);

    float accelX = deltaX * -0.15f;
    float accelY = deltaY * -0.15f;

    // Atracción hacia otras esferas cercanas
    for (const auto& pos : others) {
        float dx = pos.first - currX;
        float dy = pos.second - currY;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > 0 && dist < ATTRACTION_DISTANCE) {
            float force = ATTRACTION_STRENGTH / (dist * dist + 1.0f);
            accelX += (dx / dist) * force;
            accelY += (dy / dist) * force;
        }
    }

    sphere.velX += accelX;
    sphere.velY += accelY;
    sphere.velX += -sphere.velX * 0.25f;
    sphere.velY += -sphere.velY * 0.25f;
    sphere.velX *= 0.80f;
    sphere.velY *= 0.80f;

    float force = sqrtf(deltaX * deltaX + deltaY * deltaY);
    if (force > 0.5f) {
        float dirX = deltaX / force;
        float dirY = deltaY / force;
        sphere.stretchVelX -= dirX * force * 0.003f;
        sphere.stretchVelY -= dirY * force * 0.003f;
    }

    sphere.stretchX += sphere.stretchVelX;
    sphere.stretchY += sphere.stretchVelY;
    sphere.stretchVelX += (1.0f - sphere.stretchX) * 0.20f;
    sphere.stretchVelY += (1.0f - sphere.stretchY) * 0.20f;
    sphere.stretchVelX *= 0.75f;
    sphere.stretchVelY *= 0.75f;
    sphere.stretchX = fmaxf(0.85f, fminf(1.15f, sphere.stretchX));
    sphere.stretchY = fmaxf(0.85f, fminf(1.15f, sphere.stretchY));

    sphere.rotationY += deltaX * 0.003f;
    sphere.rotationX += deltaY * 0.003f;
    sphere.rotationY += dt * 0.15f;

    sphere.time += dt;
    sphere.breathe = 1.0f + sinf(sphere.time * 1.2f) * 0.05f;

    sphere.auroraPhase += dt * 1.2f;
    float movement = sqrtf(deltaX * deltaX + deltaY * deltaY);
    sphere.auroraIntensity += movement * 0.08f;
    sphere.auroraIntensity *= 0.92f;
    sphere.auroraIntensity = fminf(sphere.auroraIntensity, 4.0f);

    if (sphere.isForming) {
        sphere.formationProgress += dt * 0.4f;
        if (sphere.formationProgress >= 1.0f) {
            sphere.formationProgress = 1.0f;
            sphere.isForming = false;
        }
    }

    for (size_t i = 0; i < sphere.points.size(); i++) {
        Point3D& p = sphere.points[i];
        float wave = sinf(sphere.time * p.speed + p.phase) * 2.0f;
        float wave2 = cosf(sphere.time * p.speed * 1.3f + p.phase) * 2.0f;

        float targetX = (p.originalX * sphere.stretchX + wave) * sphere.breathe;
        float targetY = (p.originalY * sphere.stretchY + wave2) * sphere.breathe;
        float targetZ = p.originalZ * sphere.breathe;

        float pointDelay = (float)i / NUM_POINTS * 0.3f;
        float pointProgress = fmaxf(0.0f, fminf(1.0f, (sphere.formationProgress - pointDelay) / 0.7f));
        pointProgress = 1.0f - (1.0f - pointProgress) * (1.0f - pointProgress);

        p.x = p.x + (targetX - p.x) * pointProgress * 0.15f;
        p.y = p.y + (targetY - p.y) * pointProgress * 0.15f;
        p.z = p.z + (targetZ - p.z) * pointProgress * 0.15f;
    }

    // Efecto de ciclo de color RGB
    float colorTime = sphere.time * 0.4f;
    sphere.r = (int)(155.0f + 100.0f * sinf(colorTime));
    sphere.g = (int)(155.0f + 100.0f * sinf(colorTime + 2.0f * ALLEGRO_PI / 3.0f));
    sphere.b = (int)(155.0f + 100.0f * sinf(colorTime + 4.0f * ALLEGRO_PI / 3.0f));
}

void drawSphere(Sphere& sphere, float scale = 1.0f, float offsetX = 0.0f, float offsetY = 0.0f) {
    float centerX = (sphere.centerX + sphere.velX) * scale + offsetX;
    float centerY = (sphere.centerY + sphere.velY) * scale + offsetY;

    // 1. Resplandor Exterior Ultra-fluido y con RGB dinámico (MUY DIFUMINADO)
    if (sphere.formationProgress > 0.1f) {
        float outerGlowAlpha = std::min(1.0f, (sphere.formationProgress - 0.1f) / 0.9f);
        // Aumentado a 120 capas para un degradado extremadamente suave
        for (int glow = 120; glow > 0; glow--) {
            float t = (float)glow / 120.0f; // Normalizado 0-1
            // Radio más extendido y con una curva de potencia más suave (powf t, 0.4)
            float glowRadius = SPHERE_RADIUS * 0.5f + powf(t, 0.4f) * 300.0f;
            // Alfa con caída más suave (powf 1-t, 2.0) y multiplicador ajustado
            int alpha = clampInt((int)(powf(1.0f - t, 2.0f) * 18.0f * outerGlowAlpha));

            // El color del resplandor es RGB y tiene un desfase de tono radial
            float colorTime = sphere.time * 0.4f;
            float hueShift = t * 1.5f;
            int r = (int)(155.0f + 100.0f * sinf(colorTime + hueShift));
            int g = (int)(155.0f + 100.0f * sinf(colorTime + hueShift + 2.0f * ALLEGRO_PI / 3.0f));
            int b = (int)(155.0f + 100.0f * sinf(colorTime + hueShift + 4.0f * ALLEGRO_PI / 3.0f));

            al_draw_filled_circle(centerX, centerY, glowRadius * scale, al_map_rgba(r, g, b, alpha));
        }
    }

    // 2. Tentáculos (sin cambios, se mantienen)
    if (sphere.formationProgress > 0.25f) {
        float auroraAlpha = std::min(1.0f, (sphere.formationProgress - 0.25f) / 0.75f);
        for (size_t i = 0; i < sphere.auroraStrands.size(); i++) {
            AuroraStrand& strand = sphere.auroraStrands[i];
            int segments = 30;
            std::vector<float> pointsX(segments), pointsY(segments);
            for (int seg = 0; seg < segments; seg++) {
                float t = (float)seg / (segments - 1);
                float angle = strand.baseAngle + sphere.auroraPhase * 0.1f;
                float radius = SPHERE_RADIUS + t * strand.length;

                float noiseX = simpleNoise(t * 2.0f + sphere.time * 0.3f + strand.noiseOffset,
                                          sphere.time * 0.5f);
                float noiseY = simpleNoise(t * 3.0f + sphere.time * 0.4f + strand.noiseOffset + 10.0f,
                                          sphere.time * 0.6f);

                float wave1 = sinf(t * 4.0f + sphere.time * strand.speed + strand.phase) * strand.amplitude * t;
                float wave2 = cosf(t * 6.0f + sphere.time * strand.speed * 0.7f) * strand.amplitude * 0.3f * t;
                float curtainWave = sinf(t * 1.5f + sphere.time * 0.4f + strand.phase) * 40.0f * t * t;
                float movementWave = sphere.auroraIntensity * sinf(angle * 3.0f + sphere.time * 1.5f) * t * 12.0f;

                float totalWave = wave1 + wave2 + curtainWave + movementWave + noiseX * 25.0f * t;
                float totalWavePerp = noiseY * 15.0f * t;

                float x = centerX + cosf(angle) * radius * scale + sinf(angle + ALLEGRO_PI * 0.5f) * totalWave * scale;
                float y = centerY + sinf(angle) * radius * scale + cosf(angle + ALLEGRO_PI * 0.5f) * totalWave * scale + totalWavePerp * scale;

                pointsX[seg] = x;
                pointsY[seg] = y;
            }

            for (int seg = 0; seg < segments - 1; seg++) {
                float t = (float)seg / (segments - 1);
                float alphaFade = (1.0f - powf(t, 0.4f)) * (0.75f + 0.25f * sinf(sphere.time * 1.5f + t * 3.0f));
                int alpha = clampInt((int)(150 * alphaFade * auroraAlpha), 0, 210);

                float smoothT = t * t * (3.0f - 2.0f * t);

                float colorPhase = strand.colorShift + sphere.time * 0.15f + smoothT * 0.8f;
                float colorWave = (sinf(colorPhase) + 1.0f) * 0.5f;

                // Color opuesto al del núcleo para contraste
                int opposite_r = 255 - sphere.r;
                int opposite_g = 255 - sphere.g;
                int opposite_b = 255 - sphere.b;

                // Usar el color opuesto como base y añadirle una variación
                int r = clampInt(opposite_r + (int)(colorWave * 100) - 50);
                int g = clampInt(opposite_g + (int)(colorWave * 100) - 50);
                int b = clampInt(opposite_b + (int)(colorWave * 100) - 50);

                float thickness = (2.2f - smoothT * 1.7f) * (1.0f + sinf(sphere.time * 0.8f + t * 2.0f) * 0.15f);

                al_draw_line(pointsX[seg], pointsY[seg], pointsX[seg + 1], pointsY[seg + 1],
                            al_map_rgba(r, g, b, alpha), thickness);

                if (t < 0.4f) {
                    int glowAlpha = clampInt((int)(alpha * 0.4f * (1.0f - t * 2.5f)), 0, 255);
                    al_draw_line(pointsX[seg], pointsY[seg], pointsX[seg + 1], pointsY[seg + 1],
                                al_map_rgba(r + 50, g + 50, b + 50, glowAlpha), thickness + 1.5f);
                }
            }
        }
    }

    // 3. Núcleo Difuminado (ahora igualmente difuminado que el resplandor)
    float coreFormAlpha = powf(sphere.formationProgress, 2.0f);
    // Aumentado a 80 capas para un núcleo muy suave
    for (int i = 80; i > 0; i--) {
        float t = (float)i / 80.0f;
        float radius = SPHERE_RADIUS * 0.95f * t; // Radio lineal hasta el borde
        // Alfa similar al resplandor pero más concentrado en el centro
        int alpha = clampInt((int)(powf(1.0f - t, 2.0f) * 40.0f * coreFormAlpha));

        // El color del núcleo cicla RGB y se vuelve blanco en el centro
        float colorTime = sphere.time * 0.4f;
        int r = (int)(155.0f + 100.0f * sinf(colorTime));
        int g = (int)(155.0f + 100.0f * sinf(colorTime + 2.0f * ALLEGRO_PI / 3.0f));
        int b = (int)(155.0f + 100.0f * sinf(colorTime + 4.0f * ALLEGRO_PI / 3.0f));

        // Se hace más blanco y brillante hacia el centro (t se acerca a 0)
        r = clampInt(r + (int)(120 * (1.0f - t)));
        g = clampInt(g + (int)(120 * (1.0f - t)));
        b = clampInt(b + (int)(120 * (1.0f - t)));

        al_draw_filled_circle(centerX, centerY, radius * scale, al_map_rgba(r, g, b, alpha));
    }

    // 4. Red de Puntos y Conexiones (Restaurado y más visible)
    std::vector<DrawPoint> drawPoints;
    for (const auto& p : sphere.points) {
        DrawPoint dp;
        dp.rotated = rotatePoint(p, sphere.rotationX, sphere.rotationY);
        dp.z = dp.rotated.z;
        float perspective = 350.0f / (350.0f + dp.z);
        dp.screenX = centerX + dp.rotated.x * perspective * scale;
        dp.screenY = centerY + dp.rotated.y * perspective * scale;
        drawPoints.push_back(dp);
    }

    std::sort(drawPoints.begin(), drawPoints.end());

    float pointAlpha = powf(sphere.formationProgress, 2.0f);

    // Dibujar Conexiones (Color blanco para visibilidad)
    for (size_t i = 0; i < drawPoints.size(); ++i) {
        for (size_t j = i + 1; j < drawPoints.size(); ++j) {
            float dx = drawPoints[i].screenX - drawPoints[j].screenX;
            float dy = drawPoints[i].screenY - drawPoints[j].screenY;
            float dist = sqrtf(dx * dx + dy * dy);

            if (dist < CONNECTION_DISTANCE * scale) {
                float avgZ = (drawPoints[i].z + drawPoints[j].z) / 2.0f;
                float alpha_conn = (1.0f - dist / (CONNECTION_DISTANCE * scale));
                alpha_conn *= (1.0f - fmaxf(0, fminf(1, avgZ / SPHERE_RADIUS))); // Fade with Z
                // Alfa aumentado para mayor visibilidad
                int final_alpha = clampInt((int)(alpha_conn * 150.0f * pointAlpha));

                if (final_alpha > 0) {
                    // Color blanco brillante para destacar
                    al_draw_line(drawPoints[i].screenX, drawPoints[i].screenY,
                                 drawPoints[j].screenX, drawPoints[j].screenY,
                                 al_map_rgba(255, 255, 255, final_alpha), 1.2f); // Grosor aumentado
                }
            }
        }
    }

    // Dibujar Puntos (Color blanco para visibilidad)
    for (const auto& dp : drawPoints) {
        // Radio aumentado para mayor visibilidad
        float radius = fmaxf(0.5f, (1.0f - fmaxf(0, fminf(1, dp.z / SPHERE_RADIUS))) * 2.8f * scale);
        // Alfa aumentado
        int final_alpha = clampInt((int)(220.0f * pointAlpha));
        
        if (final_alpha > 0) {
            // Color blanco brillante para destacar
            al_draw_filled_circle(dp.screenX, dp.screenY, radius, al_map_rgba(255, 255, 255, final_alpha));
            // Resplandor suave para los puntos
            al_draw_filled_circle(dp.screenX, dp.screenY, radius * 2.0, al_map_rgba(255, 255, 255, final_alpha / 4));
        }
    }
}

int main() {
    srand((unsigned)time(nullptr));

    if (!al_init()) {
        std::cerr << "No se pudo iniciar Allegro\n";
        return -1;
    }
    al_init_primitives_addon();

    ALLEGRO_DISPLAY* display = al_create_display(SCREEN_W, SCREEN_H);
    if (!display) {
        std::cerr << "No se pudo crear la ventana\n";
        return -1;
    }
    al_set_window_title(display, "Esferas Neurales Conectadas");
    al_set_blender(ALLEGRO_ADD, ALLEGRO_ALPHA, ALLEGRO_INVERSE_ALPHA);

    ALLEGRO_TIMER* timer = al_create_timer(1.0 / 60.0);
    ALLEGRO_EVENT_QUEUE* queue = al_create_event_queue();
    al_register_event_source(queue, al_get_display_event_source(display));
    al_register_event_source(queue, al_get_timer_event_source(timer));

    Sphere sphere;
    initSphere(sphere, 0);

    int prevWX, prevWY;
    al_get_window_position(display, &prevWX, &prevWY);

    al_start_timer(timer);

    bool running = true;
    bool redraw = true;
    const float dt = 1.0f / 60.0f;

    while (running) {
        ALLEGRO_EVENT event;
        al_wait_for_event(queue, &event);

        if (event.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
            running = false;
        } else if (event.type == ALLEGRO_EVENT_TIMER) {
            redraw = true;

            int currWX, currWY;
            al_get_window_position(display, &currWX, &currWY);

            // Update the single sphere based on window movement
            updateSphere(sphere, prevWX, prevWY, currWX, currWY, dt, {});

            prevWX = currWX;
            prevWY = currWY;
        }

        if (redraw && al_is_event_queue_empty(queue)) {
            redraw = false;
            al_clear_to_color(al_map_rgb(8, 8, 10));

            drawSphere(sphere);

            al_flip_display();
        }
    }

    al_destroy_display(display);
    al_destroy_timer(timer);
    al_destroy_event_queue(queue);
    return 0;
}
