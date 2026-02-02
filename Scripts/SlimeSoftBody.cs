using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class SlimeSoftBody : MonoBehaviour
{
    // ================= 视觉设置 =================
    [Header("视觉表现 (Visuals)")]
    public Transform leftEyeAnchor; public Transform rightEyeAnchor;
    public Transform noseAnchor; public Transform leftEarAnchor;
    public Transform rightEarAnchor; public Transform tailAnchor;
    public Transform legFrontLeft; public Transform legFrontRight;
    public Transform legBackLeft; public Transform legBackRight;

    [Header("颜色设置")]
    public Color baseColor = new Color(1f, 0.75f, 0.8f);
    public Color inflatedColor = new Color(1f, 0.92f, 0.94f);

    // ================= 控制设置 =================
    [Header("双人控制设置")]
    public string moveAxisName = "Horizontal";
    public bool useGamepadJump = false;
    public KeyCode keyboardJumpKey = KeyCode.Space;
    public string gamepadJumpButton = "joystick button 0";

    // ================= 跨角色碰撞设置 =================
    [Header("玩家间碰撞 (Player-to-Player)")]
    public SlimeSoftBody otherPlayer;
    [Tooltip("碰撞皮肤厚度：决定贴得有多近，建议 0.1 - 0.2")]
    public float collisionSkin = 0.15f;
    [Tooltip("碰撞排斥硬度：建议 0.2 - 0.5")]
    public float playerCollisionStiffness = 0.3f;

    // ================= 模拟设置 =================
    [Header("模拟精度")]
    public int nodeCount = 40;
    public float radius = 1.0f;
    public int constraintIterations = 6;

    [Header("物理属性")]
    public Vector2 gravity = new Vector2(0, -35.0f);
    public float damping = 0.98f;
    public float jumpForce = 650f;
    public float moveSpeed = 35f;
    public float maxVelocity = 1.5f;
    public float maxVelocityInflated = 0.15f;
    public float uprightForce = 15.0f;
    public float pressureDamping = 10.0f;

    [Header("充气限制")]
    public float targetArea = 5.0f;
    public float pressureMultiplier = 400f;
    public float minArea = 2.0f;
    public float maxArea = 25.0f;

    // ================= 内部状态 =================
    public class Node { public Vector2 pos; public Vector2 prevPos; public Vector2 acceleration; }
    public List<Node> nodes = new List<Node>();
    private float[] targetDistances;
    private float[] originalAngles;
    private SimpleWall[] allWalls;
    private Mesh mesh;
    private Vector3[] vertices;
    private int[] triangles;
    private Material bodyMaterial;
    private bool isGrounded = false;
    private float currentArea;
    private int leftEyeIdx, rightEyeIdx, noseIdx, leftEarIdx, rightEarIdx, tailIdx, legFLIdx, legFRIdx, legBLIdx, legBRIdx;
    private SimpleBox[] allBoxes; // 在 Start 中获取场景内所有箱子
    void Start()
    {
        allWalls = FindObjectsOfType<SimpleWall>();
        nodes = new List<Node>();
        targetDistances = new float[nodeCount];
        originalAngles = new float[nodeCount];
        allWalls = FindObjectsOfType<SimpleWall>();
        allBoxes = FindObjectsOfType<SimpleBox>(); // 获取所有物理箱子

        for (int i = 0; i < nodeCount; i++)
        {
            float angle = i * 2 * Mathf.PI / nodeCount;
            Vector2 p = (Vector2)transform.position + new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * radius;
            nodes.Add(new Node { pos = p, prevPos = p, acceleration = Vector2.zero });
            originalAngles[i] = angle;
        }

        for (int i = 0; i < nodeCount; i++) targetDistances[i] = Vector2.Distance(nodes[i].pos, nodes[(i + 1) % nodeCount].pos);

        mesh = new Mesh(); GetComponent<MeshFilter>().mesh = mesh;
        bodyMaterial = GetComponent<MeshRenderer>().material;

        noseIdx = 0; leftEarIdx = nodeCount / 8; rightEarIdx = nodeCount - (nodeCount / 8);
        leftEyeIdx = nodeCount / 10; rightEyeIdx = nodeCount - (nodeCount / 10);
        tailIdx = nodeCount / 2; legFLIdx = 15; legFRIdx = 25; legBLIdx = 18; legBRIdx = 22;
    }

    void Update() { HandleInput(); UpdateVisuals(); }

    void FixedUpdate() { SimulatePhysics(Time.fixedDeltaTime); }

    void HandleInput()
    {
        float h = Input.GetAxis(moveAxisName);
        if (Mathf.Abs(h) > 0.01f) foreach (var n in nodes) n.acceleration.x += h * moveSpeed;

        bool wantsToJump = useGamepadJump ? Input.GetKeyDown(gamepadJumpButton) : Input.GetKeyDown(keyboardJumpKey);
        if (wantsToJump && isGrounded)
        {
            float t = Mathf.InverseLerp(minArea, maxArea, currentArea);
            float jumpMultiplier = Mathf.Lerp(1.0f, 0.3f, t);
            foreach (var n in nodes) n.prevPos.y -= (jumpForce * jumpMultiplier) * 0.001f;
        }
    }

    void SimulatePhysics(float dt)
    {
        currentArea = CalculateArea();
        if (float.IsNaN(currentArea)) return;

        isGrounded = false;
        float areaT = Mathf.InverseLerp(minArea, maxArea, currentArea);
        float dynMaxVel = Mathf.Lerp(maxVelocity, maxVelocityInflated, areaT);
        Vector2 curGravity = gravity * Mathf.Lerp(1.5f, -0.5f, areaT);

        float pressure = Mathf.Clamp((targetArea - currentArea) * pressureMultiplier, -400f, 400f);
        Vector2 center = Vector2.zero;
        foreach (var n in nodes) center += n.pos;
        center /= nodeCount;
        transform.position = center;

        Vector2[] forces = new Vector2[nodeCount];
        Vector2 netForce = Vector2.zero;
        System.Array.Clear(forces, 0, nodeCount);

        for (int i = 0; i < nodeCount; i++)
        {
            Vector2 edge = nodes[(i + 1) % nodeCount].pos - nodes[i].pos;
            Vector2 normal = new Vector2(-edge.y, edge.x).normalized;
            forces[i] += normal * pressure * 0.5f;
            forces[(i + 1) % nodeCount] += normal * pressure * 0.5f;
        }

        for (int i = 0; i < nodeCount; i++)
        {
            Vector2 r = nodes[i].pos - center, vel = nodes[i].pos - nodes[i].prevPos;
            if (r.sqrMagnitude > 0.001f)
            {
                forces[i] += -r.normalized * Vector2.Dot(vel, r.normalized) * pressureDamping;
                float angleErr = Mathf.DeltaAngle(Mathf.Atan2(r.y, r.x) * Mathf.Rad2Deg, originalAngles[i] * Mathf.Rad2Deg);
                forces[i] += new Vector2(-r.y, r.x).normalized * (angleErr * 0.01f) * uprightForce;
            }
            netForce += forces[i];
        }

        Vector2 correction = netForce / nodeCount;
        for (int i = 0; i < nodeCount; i++)
        {
            Node n = nodes[i];
            n.acceleration += (forces[i] - correction) + curGravity;
            Vector2 temp = n.pos;
            Vector2 disp = (n.pos - n.prevPos) * damping + n.acceleration * dt * dt;
            if (disp.magnitude > dynMaxVel) disp = disp.normalized * dynMaxVel;
            n.pos += disp; n.prevPos = temp; n.acceleration = Vector2.zero;
        }

        for (int k = 0; k < constraintIterations; k++)
        {
            bool fw = (k % 2 == 0);
            for (int i = 0; i < nodeCount; i++) SolveSpring(fw ? i : (nodeCount - 1 - i));
            ApplyCollisions();
            ApplyPlayerCollision();
        }
    }

    void ApplyPlayerCollision()
    {
        if (otherPlayer == null) return;

        var otherNodes = otherPlayer.nodes;
        int otherCnt = otherNodes.Count;

        foreach (var myNode in nodes)
        {
            float minDistance = float.MaxValue;
            Vector2 pushDir = Vector2.zero;
            bool isInside = true;

            for (int i = 0; i < otherCnt; i++)
            {
                Vector2 a = otherNodes[i].pos;
                Vector2 b = otherNodes[(i + 1) % otherCnt].pos;

                Vector2 edge = b - a;
                Vector2 normal = new Vector2(-edge.y, edge.x).normalized;

                float distToEdge = Vector2.Dot(myNode.pos - a, normal);
                if (distToEdge > 0) isInside = false;

                Vector2 closest = GetClosestPointOnSegment(a, b, myNode.pos);
                float d = Vector2.Distance(myNode.pos, closest);
                if (d < minDistance)
                {
                    minDistance = d;
                    pushDir = myNode.pos - closest;
                }
            }

            if (isInside || minDistance < collisionSkin)
            {
                float currentDist = pushDir.magnitude;
                float overlap = collisionSkin - (isInside ? -currentDist : currentDist);

                Vector2 resolveDir = isInside ? -pushDir.normalized : pushDir.normalized;
                if (pushDir.sqrMagnitude < 0.0001f) resolveDir = (myNode.pos - (Vector2)otherPlayer.transform.position).normalized;

                myNode.pos += resolveDir * overlap * playerCollisionStiffness;

                // --- 新增判定：踩头跳 ---
                // 如果碰撞发生，且我的质点在对方中心点上方，则判定为“着地”
                if (myNode.pos.y > otherPlayer.transform.position.y)
                {
                    isGrounded = true;
                }
            }
        }
    }

    Vector2 GetClosestPointOnSegment(Vector2 a, Vector2 b, Vector2 p)
    {
        Vector2 ap = p - a; Vector2 ab = b - a;
        float t = Mathf.Clamp01(Vector2.Dot(ap, ab) / ab.sqrMagnitude);
        return a + ab * t;
    }

    public List<Vector2> GetNodePositions()
    {
        List<Vector2> posList = new List<Vector2>();
        foreach (var n in nodes) posList.Add(n.pos);
        return posList;
    }

    public void AddGlobalForce(Vector2 force)
    {
        Vector2 fPerNode = force / nodeCount;
        foreach (var n in nodes) n.acceleration += fPerNode;
    }

    void SolveSpring(int i)
    {
        Node n1 = nodes[i], n2 = nodes[(i + 1) % nodeCount];
        Vector2 delta = n2.pos - n1.pos; float dist = delta.magnitude;
        if (dist < 0.001f) return;
        Vector2 c = delta * 0.5f * (dist - targetDistances[i]) / dist;
        n1.pos += c; n2.pos -= c;
    }

    void ApplyCollisions()
    {
        if (allWalls == null) return;
        foreach (var node in nodes)
        {
            Vector2 vel = node.pos - node.prevPos;
            foreach (var wall in allWalls)
            {
                Bounds b = wall.GetBounds();
                if (node.pos.x > b.min.x && node.pos.x < b.max.x && node.pos.y > b.min.y && node.pos.y < b.max.y)
                {
                    float dL = node.pos.x - b.min.x, dR = b.max.x - node.pos.x, dB = node.pos.y - b.min.y, dT = b.max.y - node.pos.y;
                    float minP = Mathf.Min(dL, dR, dB, dT);
                    if (minP == dT) { node.pos.y = b.max.y; node.prevPos.y = node.pos.y; node.prevPos.x += vel.x * wall.friction; isGrounded = true; }
                    else if (minP == dB) { node.pos.y = b.min.y; node.prevPos.y = node.pos.y; node.prevPos.x += vel.x * wall.friction; }
                    else if (minP == dL) { node.pos.x = b.min.x; node.prevPos.x = node.pos.x; node.prevPos.y += vel.y * wall.friction; }
                    else if (minP == dR) { node.pos.x = b.max.x; node.prevPos.x = node.pos.x; node.prevPos.y += vel.y * wall.friction; }
                }
            }
        }

        SimpleBox[] allBoxes = FindObjectsOfType<SimpleBox>();
        foreach (var box in allBoxes)
        {
            // 计算箱子的旋转矩阵
            float cos = Mathf.Cos(-box.angle);
            float sin = Mathf.Sin(-box.angle);
            Vector2 halfSize = box.size * 0.5f;

            foreach (var node in nodes)
            {
                // 1. 将质点转为箱子的本地坐标 (Local Space)
                Vector2 relativePos = node.pos - box.position;
                Vector2 localPos = new Vector2(
                    relativePos.x * cos - relativePos.y * sin,
                    relativePos.x * sin + relativePos.y * cos
                );

                // 2. 在本地坐标下进行矩形碰撞检测
                if (Mathf.Abs(localPos.x) < halfSize.x && Mathf.Abs(localPos.y) < halfSize.y)
                {
                    // 计算挤出向量 (Local Space)
                    float dx = halfSize.x - Mathf.Abs(localPos.x);
                    float dy = halfSize.y - Mathf.Abs(localPos.y);

                    Vector2 localNormal;
                    if (dx < dy)
                    {
                        localNormal = new Vector2(Mathf.Sign(localPos.x), 0);
                        localPos.x = halfSize.x * Mathf.Sign(localPos.x);
                    }
                    else
                    {
                        localNormal = new Vector2(0, Mathf.Sign(localPos.y));
                        localPos.y = halfSize.y * Mathf.Sign(localPos.y);
                        if (localNormal.y > 0) isGrounded = true; // 踩在顶面
                    }

                    // 3. 将位置和法线转回世界坐标 (World Space)
                    float cosW = Mathf.Cos(box.angle);
                    float sinW = Mathf.Sin(box.angle);
                    node.pos = new Vector2(
                        box.position.x + localPos.x * cosW - localPos.y * sinW,
                        box.position.y + localPos.x * sinW + localPos.y * cosW
                    );

                    Vector2 worldNormal = new Vector2(
                        localNormal.x * cosW - localNormal.y * sinW,
                        localNormal.x * sinW + localNormal.y * cosW
                    );

                    // 4. 应用冲量到箱子
                    // 关键：冲量的大小取决于碰撞深度和猪猪的内压
                    float impulseMagnitude = Mathf.Min(dx, dy) * 0.5f;
                    Vector2 impulse = -worldNormal * impulseMagnitude;

                    // 作用点就是当前质点的位置，这样偏心碰撞就会产生旋转！
                    box.AddImpulseAtPosition(impulse, node.pos);

                    // 5. 摩擦力反馈
                    Vector2 vel = node.pos - node.prevPos;
                    node.prevPos += vel * box.friction;
                }
            }
        }

        // 在 ApplyCollisions 内部添加
        // --- 增强型跷跷板碰撞逻辑 (解决隧道效应) ---
        SimpleSeesaw[] allSeesaws = FindObjectsOfType<SimpleSeesaw>();
        foreach (var seesaw in allSeesaws)
        {
            float cos = Mathf.Cos(-seesaw.angle);
            float sin = Mathf.Sin(-seesaw.angle);
            Vector2 halfSize = seesaw.size * 0.5f;
            Vector2 pivotPos = seesaw.transform.position;

            // 增加一个“安全厚度”缓冲区
            float safetyBuffer = 0.3f;

            foreach (var node in nodes)
            {
                // 1. 获取当前位置的本地坐标
                Vector2 relativePos = node.pos - pivotPos;
                Vector2 localPos = new Vector2(
                    relativePos.x * cos - relativePos.y * sin,
                    relativePos.x * sin + relativePos.y * cos
                );

                // 2. 获取上一帧位置的本地坐标 (关键！)
                Vector2 prevRelativePos = node.prevPos - pivotPos;
                Vector2 prevLocalPos = new Vector2(
                    prevRelativePos.x * cos - prevRelativePos.y * sin,
                    prevRelativePos.x * sin + prevRelativePos.y * cos
                );

                // 3. 碰撞判定改进：
                // A. 质点当前就在跷跷板内
                // B. 或者，质点在本帧穿过了跷跷板的中心平面 (从 y>0 变到 y<0，或反之)
                bool isInside = Mathf.Abs(localPos.x) < halfSize.x && Mathf.Abs(localPos.y) < halfSize.y;
                bool didCross = Mathf.Abs(localPos.x) < halfSize.x && (Mathf.Sign(localPos.y) != Mathf.Sign(prevLocalPos.y)) && (Mathf.Abs(prevLocalPos.y) < 1.0f);

                if (isInside || didCross)
                {
                    // 确定挤出方向：如果穿透了，则根据上一帧位置挤回原侧
                    float pushSign = (prevLocalPos.y > 0) ? 1 : -1;

                    // 强制将 localPos 修正到表面
                    localPos.y = halfSize.y * pushSign;

                    if (pushSign > 0) isGrounded = true; // 踩在上面

                    // 4. 将修正后的本地位置转回世界坐标
                    float cosW = Mathf.Cos(seesaw.angle);
                    float sinW = Mathf.Sin(seesaw.angle);
                    node.pos = new Vector2(
                        pivotPos.x + localPos.x * cosW - localPos.y * sinW,
                        pivotPos.y + localPos.x * sinW + localPos.y * cosW
                    );

                    // 5. 施加力矩
                    // 冲量计算：基于位置的变化量
                    Vector2 impulse = (node.prevPos - node.pos) * 0.5f;
                    seesaw.AddImpulseAtPosition(impulse, node.pos);
                }
            }
        }
    }

    // 将这些放在 SlimeSoftBody.cs 类的末尾

    public void SetGrounded(bool state)
    {
        isGrounded = state;
    }

    public Vector2 GetAverageVelocity()
    {
        if (nodes == null || nodes.Count == 0) return Vector2.zero;
        Vector2 avg = Vector2.zero;
        foreach (var n in nodes) avg += (n.pos - n.prevPos);
        return (avg / nodes.Count) / Time.fixedDeltaTime;
    }

    float CalculateArea()
    {
        float a = 0f;
        for (int i = 0; i < nodeCount; i++) a += (nodes[i].pos.x * nodes[(i + 1) % nodeCount].pos.y - nodes[(i + 1) % nodeCount].pos.x * nodes[i].pos.y);
        return Mathf.Abs(a * 0.5f);
    }

    void UpdateVisuals()
    {
        bodyMaterial.color = Color.Lerp(baseColor, inflatedColor, Mathf.InverseLerp(minArea, maxArea, currentArea));
        if (vertices == null) vertices = new Vector3[nodeCount + 1];
        vertices[0] = transform.InverseTransformPoint(transform.position);
        for (int i = 0; i < nodeCount; i++) vertices[i + 1] = transform.InverseTransformPoint(nodes[i].pos);
        if (triangles == null)
        {
            triangles = new int[nodeCount * 3];
            for (int i = 0; i < nodeCount; i++) { triangles[i * 3] = 0; triangles[i * 3 + 1] = i + 1; triangles[i * 3 + 2] = (i + 1) % nodeCount + 1; }
        }
        mesh.vertices = vertices; mesh.triangles = triangles; mesh.RecalculateBounds();
        Vector2 c = transform.position;
        UpdateP(leftEyeAnchor, c, leftEyeIdx, 0.8f, false); UpdateP(rightEyeAnchor, c, rightEyeIdx, 0.8f, false);
        UpdateP(noseAnchor, c, noseIdx, 0.7f, false); UpdateP(leftEarAnchor, c, leftEarIdx, 1.0f, false);
        UpdateP(rightEarAnchor, c, rightEarIdx, 1.0f, false); UpdateP(tailAnchor, c, tailIdx, 1.0f, false);
        UpdateP(legFrontLeft, c, legFLIdx, 1.0f, true); UpdateP(legFrontRight, c, legFRIdx, 1.0f, true);
        UpdateP(legBackLeft, c, legBLIdx, 1.0f, true); UpdateP(legBackRight, c, legBRIdx, 1.0f, true);
    }

    void UpdateP(Transform t, Vector2 c, int i, float l, bool v)
    {
        if (t == null) return; t.position = Vector2.Lerp(c, nodes[i].pos, l);
        t.up = v ? Vector2.up : (nodes[i].pos - c).normalized;
    }
}