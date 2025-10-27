#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <chrono>
#include <thread>

// libnifalcon
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/comm/FalconCommLibUSB.h>

// tinyobjloader
#define TINYOBJLOADER_IMPLEMENTATION
#include "external/tiny_obj_loader.h"

using namespace libnifalcon;

// ========================================
// 3Dベクトル構造体
// ========================================
struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }

    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalize() const {
        double len = length();
        return len > 0.0001 ? (*this / len) : Vec3(0, 0, 0);
    }
};

// ========================================
// 三角形構造体
// ========================================
struct Triangle {
    Vec3 v0, v1, v2;  // 頂点
    Vec3 normal;      // 面法線

    Triangle(const Vec3& _v0, const Vec3& _v1, const Vec3& _v2)
        : v0(_v0), v1(_v1), v2(_v2) {
        // 面法線を計算（右手系）
        Vec3 edge1 = v1 - v0;
        Vec3 edge2 = v2 - v0;
        normal = edge1.cross(edge2).normalize();
    }
};

// ========================================
// OBJメッシュローダー
// ========================================
bool loadOBJMesh(const std::string& filename, std::vector<Triangle>& mesh, double scale = 1.0) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    std::cout << "OBJファイルを読み込んでいます: " << filename << std::endl;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str());

    if (!warn.empty()) {
        std::cout << "警告: " << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << "エラー: " << err << std::endl;
    }

    if (!ret) {
        std::cerr << "OBJファイルの読み込みに失敗しました。" << std::endl;
        return false;
    }

    std::cout << "頂点数: " << attrib.vertices.size() / 3 << std::endl;
    std::cout << "シェイプ数: " << shapes.size() << std::endl;

    // 三角形メッシュを構築
    mesh.clear();
    for (size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];

            // 三角形のみをサポート
            if (fv != 3) {
                std::cerr << "警告: 三角形以外の面が見つかりました（頂点数: " << fv << "）" << std::endl;
                index_offset += fv;
                continue;
            }

            // 3つの頂点を取得
            Vec3 vertices[3];
            for (int v = 0; v < 3; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                vertices[v] = Vec3(
                    attrib.vertices[3 * idx.vertex_index + 0] * scale,
                    attrib.vertices[3 * idx.vertex_index + 1] * scale,
                    attrib.vertices[3 * idx.vertex_index + 2] * scale
                );
            }

            mesh.push_back(Triangle(vertices[0], vertices[1], vertices[2]));
            index_offset += fv;
        }
    }

    std::cout << "三角形数: " << mesh.size() << std::endl;
    return true;
}

// ========================================
// 点と三角形の最短距離を計算
// ========================================
struct CollisionResult {
    bool collision;       // 衝突したか
    bool inside;          // メッシュの内側にいるか
    double distance;      // 最短距離
    Vec3 closestPoint;    // 最近接点
    Vec3 normal;          // 法線（力を加える方向）
};

// 点と三角形の最短距離を計算
CollisionResult pointTriangleDistance(const Vec3& p, const Triangle& tri) {
    CollisionResult result;
    result.collision = false;
    result.inside = false;
    result.distance = std::numeric_limits<double>::max();

    Vec3 v0 = tri.v0;
    Vec3 v1 = tri.v1;
    Vec3 v2 = tri.v2;

    Vec3 edge0 = v1 - v0;
    Vec3 edge1 = v2 - v1;
    Vec3 edge2 = v0 - v2;

    Vec3 v0p = p - v0;

    // 点を三角形の平面に投影
    double distToPlane = v0p.dot(tri.normal);
    Vec3 projectedPoint = p - tri.normal * distToPlane;

    // 投影点が三角形内にあるかチェック（重心座標法）
    Vec3 v0proj = projectedPoint - v0;
    Vec3 v1proj = projectedPoint - v1;
    Vec3 v2proj = projectedPoint - v2;

    Vec3 c0 = edge0.cross(v0proj);
    Vec3 c1 = edge1.cross(v1proj);
    Vec3 c2 = edge2.cross(v2proj);

    bool projInside = (c0.dot(tri.normal) >= 0) &&
                      (c1.dot(tri.normal) >= 0) &&
                      (c2.dot(tri.normal) >= 0);

    if (projInside) {
        // 投影点が三角形内にある場合
        result.closestPoint = projectedPoint;
        result.distance = std::abs(distToPlane);

        // 点が三角形の裏側（内側）にいるか判定
        if (distToPlane < 0) {
            result.inside = true;
            result.normal = tri.normal;  // 外側向き（法線方向）
        } else {
            result.inside = false;
            result.normal = tri.normal;
        }
    } else {
        // 投影点が三角形外の場合、エッジと頂点をチェック
        double minDist = std::numeric_limits<double>::max();
        Vec3 closestPt;

        // 3つのエッジをチェック
        std::array<Vec3, 3> edges = {v0, v1, v2};
        std::array<Vec3, 3> edgeVecs = {edge0, edge1, edge2};

        for (int i = 0; i < 3; i++) {
            Vec3 edgeStart = edges[i];
            Vec3 edgeEnd = edges[(i + 1) % 3];
            Vec3 edgeVec = edgeVecs[i];

            double t = (p - edgeStart).dot(edgeVec) / edgeVec.dot(edgeVec);
            t = std::max(0.0, std::min(1.0, t));

            Vec3 pointOnEdge = edgeStart + edgeVec * t;
            double dist = (p - pointOnEdge).length();

            if (dist < minDist) {
                minDist = dist;
                closestPt = pointOnEdge;
            }
        }

        result.closestPoint = closestPt;
        result.distance = minDist;
        result.normal = (p - closestPt).normalize();
    }

    return result;
}

// ========================================
// メッシュ全体との衝突判定
// ========================================
CollisionResult checkMeshCollision(const Vec3& point, const std::vector<Triangle>& mesh) {
    CollisionResult bestResult;
    bestResult.collision = false;
    bestResult.inside = false;
    bestResult.distance = std::numeric_limits<double>::max();

    for (const auto& tri : mesh) {
        CollisionResult res = pointTriangleDistance(point, tri);

        // 最も近い三角形を見つける
        if (res.distance < bestResult.distance) {
            bestResult = res;
        }
    }

    // 衝突判定の閾値（メッシュ表面からの距離）
    const double collisionThreshold = 0.005; // 5mm（内側侵入を防ぐため大きめに）

    if (bestResult.inside) {
        // 内側にいる場合は常に衝突として扱う
        bestResult.collision = true;
    } else if (bestResult.distance < collisionThreshold) {
        // 外側で表面に近い場合も衝突
        bestResult.collision = true;
    }

    return bestResult;
}

// ========================================
// メイン関数
// ========================================
int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "  3Dメッシュ触覚フィードバック" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // コマンドライン引数からOBJファイル名を取得
    std::string objFilename = "models/test.obj";
    double scale = 0.03; // デフォルトスケール（30mm）- Falcon作業空間に合わせて小さくした

    if (argc > 1) {
        objFilename = argv[1];
    }
    if (argc > 2) {
        scale = std::stod(argv[2]);
    }

    // メッシュのオフセット位置（Falcon作業空間の下側）
    // ユーザーが上から触れるように配置
    Vec3 meshOffset(0.0, 0.0, 0.100); // (0mm, 0mm, 100mm)

    // OBJメッシュを読み込む
    std::vector<Triangle> mesh;
    if (!loadOBJMesh(objFilename, mesh, scale)) {
        std::cerr << "OBJファイルの読み込みに失敗しました。" << std::endl;
        return 1;
    }

    if (mesh.empty()) {
        std::cerr << "メッシュが空です。" << std::endl;
        return 1;
    }

    // メッシュをオフセット位置に移動
    for (auto& tri : mesh) {
        tri.v0 = tri.v0 + meshOffset;
        tri.v1 = tri.v1 + meshOffset;
        tri.v2 = tri.v2 + meshOffset;
    }

    std::cout << std::endl;
    std::cout << "モデルスケール: " << scale << std::endl;
    std::cout << "モデル位置: ["
              << (meshOffset.x * 1000) << ", "
              << (meshOffset.y * 1000) << ", "
              << (meshOffset.z * 1000) << "] mm" << std::endl;
    std::cout << std::endl;

    // Falconデバイスの初期化
    std::cout << "Falconデバイスを初期化しています..." << std::endl;
    FalconDevice falcon;
    falcon.setFalconComm<FalconCommLibUSB>();

    unsigned int count;
    falcon.getDeviceCount(count);
    std::cout << "検出されたFalconデバイス: " << count << std::endl;

    if (count == 0) {
        std::cerr << "エラー: Falconデバイスが見つかりません。" << std::endl;
        return 1;
    }

    std::cout << "デバイスを開いています..." << std::endl;
    if (!falcon.open(0)) {
        std::cerr << "エラー: デバイスを開けません。" << std::endl;
        return 1;
    }

    std::cout << "ファームウェアをロード中..." << std::endl;
    falcon.setFalconFirmware<FalconFirmwareNovintSDK>();
    if (!falcon.loadFirmware(10, false)) {
        std::cout << "警告: ファームウェアのロードに失敗しました。" << std::endl;
    }

    falcon.setFalconKinematic<FalconKinematicStamper>();

    // 力覚パラメータ
    const double stiffness = 800.0;           // 剛性 [N/m]
    const double maxForce = 3.0;              // 最大力 [N]
    const double loopFrequency = 1000.0;      // ループ周波数 [Hz]
    const double loopPeriod = 1.0 / loopFrequency;

    std::cout << std::endl;
    std::cout << "力覚パラメータ:" << std::endl;
    std::cout << "  剛性: " << stiffness << " N/m" << std::endl;
    std::cout << "  最大力: " << maxForce << " N" << std::endl;
    std::cout << "  ループ周波数: " << loopFrequency << " Hz" << std::endl;
    std::cout << std::endl;
    std::cout << "グリップを動かして3Dモデルに触れてください。" << std::endl;
    std::cout << "Ctrl+C で終了してください。" << std::endl;
    std::cout << std::endl;

    int loopCount = 0;
    bool isInitializing = true;

    while (true) {
        auto loopStart = std::chrono::high_resolution_clock::now();

        if (!falcon.runIOLoop()) {
            continue;
        }

        // Falconの現在位置を取得
        auto falconPos = falcon.getPosition();
        Vec3 pos(falconPos[0], falconPos[1], falconPos[2]);

        // 初期化フェーズ（キャリブレーション待ち）
        if (isInitializing) {
            if (loopCount == 0) {
                std::cout << "エンドエフェクタを一番外側まで動かしてください（z > 170mm）" << std::endl;
                std::cout << "その後、上から下に動かして3Dモデルに近づいてください。" << std::endl;
            }
            // 位置情報を定期的に表示
            if (loopCount % 500 == 0) {
                std::cout << "現在位置: ["
                          << (pos.x * 1000) << ", "
                          << (pos.y * 1000) << ", "
                          << (pos.z * 1000) << "] mm" << std::endl;
            }
            if (pos.z > 0.170) {
                std::cout << "メッシュ触覚シミュレーションを開始します..." << std::endl;
                std::cout << "グリップを下（Z方向の負の方向）に動かして3Dモデルに触れてください。" << std::endl;
                isInitializing = false;
            }
            loopCount++;
            continue;
        }

        // メッシュとの衝突判定
        CollisionResult collision = checkMeshCollision(pos, mesh);

        std::array<double, 3> force = {0.0, 0.0, 0.0};

        if (collision.collision) {
            double penetration = collision.distance;
            double forceMagnitude;

            if (collision.inside) {
                // 内側にいる場合：強い力で外側に押し出す
                forceMagnitude = stiffness * penetration * 3.0; // 3倍の力

                // 力の最大値を制限
                if (forceMagnitude > maxForce) {
                    forceMagnitude = maxForce;
                }

                // 法線方向（外側向き）に力を加える
                Vec3 forceVec = collision.normal * forceMagnitude;
                force[0] = forceVec.x;
                force[1] = forceVec.y;
                force[2] = forceVec.z;

                // デバッグ出力（1秒ごと）
                if (loopCount % 1000 == 0) {
                    std::cout << "内側! 位置: ["
                              << (pos.x * 1000) << ", "
                              << (pos.y * 1000) << ", "
                              << (pos.z * 1000) << "] mm | "
                              << "距離: " << (penetration * 1000) << " mm | "
                              << "力: " << forceMagnitude << " N (押し出し)"
                              << std::endl;
                }
            } else {
                // 外側から接触している場合：通常の反力
                forceMagnitude = stiffness * penetration;

                // 力の最大値を制限
                if (forceMagnitude > maxForce) {
                    forceMagnitude = maxForce;
                }

                // 法線方向に力を加える
                Vec3 forceVec = collision.normal * forceMagnitude;
                force[0] = forceVec.x;
                force[1] = forceVec.y;
                force[2] = forceVec.z;

                // デバッグ出力（1秒ごと）
                if (loopCount % 1000 == 0) {
                    std::cout << "接触! 位置: ["
                              << (pos.x * 1000) << ", "
                              << (pos.y * 1000) << ", "
                              << (pos.z * 1000) << "] mm | "
                              << "侵入: " << (penetration * 1000) << " mm | "
                              << "力: " << forceMagnitude << " N"
                              << std::endl;
                }
            }
        } else {
            // メッシュ外（力なし）
            if (loopCount % 2000 == 0) {
                std::cout << "メッシュ外: 距離 = " << (collision.distance * 1000) << " mm" << std::endl;
            }
        }

        // 力をデバイスに送信
        falcon.setForce(force);

        loopCount++;

        // ループ周期を維持（約1kHz）
        auto loopEnd = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(loopEnd - loopStart).count();
        auto sleepTime = static_cast<long>((loopPeriod * 1000000) - elapsed);
        if (sleepTime > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleepTime));
        }
    }

    falcon.close();
    return 0;
}
