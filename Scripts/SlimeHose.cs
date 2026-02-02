using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(LineRenderer))]
public class SlimeHose : MonoBehaviour
{
    [Header("连接对象")]
    public SlimeSoftBody player1;
    public SlimeSoftBody player2;

    [Header("软管物理参数")]
    [Tooltip("软管分段数：越多越平滑，但性能消耗越大")]
    public int segments = 20;
    [Tooltip("软管总长度：决定了两人之间的自然间距")]
    public float totalLength = 8.0f;
    [Tooltip("软管粗细")]
    public float hoseWidth = 0.3f;
    [Tooltip("软管重力：让它自然下垂")]
    public Vector2 gravity = new Vector2(0, -15f);
    [Tooltip("阻尼：防止软管晃动得太厉害")]
    public float damping = 0.9f;
    [Tooltip("硬度：越大越不容易被拉伸")]
    public float stiffness = 0.8f;
    [Tooltip("迭代次数：越高越稳定")]
    public int iterations = 10;

    [Header("牵引力设置")]
    [Tooltip("当距离超过长度时，对玩家施加的拉力系数")]
    public float pullForceMultiplier = 50f;

    [Header("视觉材质")]
    [Tooltip("请拖入一个材质，建议使用圆润的 Sprite 材质")]
    public Material hoseMaterial;
    public Color hoseColor = new Color(0.3f, 0.6f, 1f);

    // 内部物理结构
    private class HoseNode
    {
        public Vector2 pos, prevPos;
    }
    private List<HoseNode> nodes = new List<HoseNode>();
    private float segmentLength;
    private LineRenderer lr;

    void Start()
    {
        if (player1 == null || player2 == null)
        {
            Debug.LogError("SlimeHose: 未指定 Player1 或 Player2！");
            enabled = false;
            return;
        }

        lr = GetComponent<LineRenderer>();
        SetupLineRenderer();

        // 初始化节点
        nodes.Clear();
        Vector2 startPos = player1.transform.position;
        Vector2 endPos = player2.transform.position;
        for (int i = 0; i < segments; i++)
        {
            Vector2 p = Vector2.Lerp(startPos, endPos, (float)i / (segments - 1));
            nodes.Add(new HoseNode { pos = p, prevPos = p });
        }

        // 计算每段的理想长度
        segmentLength = totalLength / (segments - 1);
    }

    void SetupLineRenderer()
    {
        lr.positionCount = segments;
        lr.startWidth = hoseWidth;
        lr.endWidth = hoseWidth;
        lr.useWorldSpace = true;
        if (hoseMaterial != null) lr.material = hoseMaterial;
        lr.startColor = hoseColor;
        lr.endColor = hoseColor;
        // 让线条圆润一些
        lr.numCapVertices = 5;
        lr.numCornerVertices = 5;
    }

    void FixedUpdate()
    {
        if (player1 == null || player2 == null) return;

        SimulateHosePhysics(Time.fixedDeltaTime);
        ApplyPullForceToPlayers();
    }

    void Update()
    {
        UpdateVisuals();
    }

    // 核心物理模拟 (Verlet 积分)
    void SimulateHosePhysics(float dt)
    {
        // 1. 将首尾节点钉在玩家身上
        nodes[0].pos = player1.transform.position;
        nodes[segments - 1].pos = player2.transform.position;

        // 2. 更新中间节点的物理 (重力 + 惯性)
        for (int i = 1; i < segments - 1; i++)
        {
            HoseNode n = nodes[i];
            Vector2 vel = (n.pos - n.prevPos) * damping;
            n.prevPos = n.pos;
            n.pos += vel + gravity * dt * dt;
        }

        // 3. 约束求解 (保持节段长度)
        for (int k = 0; k < iterations; k++)
        {
            for (int i = 0; i < segments - 1; i++)
            {
                HoseNode n1 = nodes[i];
                HoseNode n2 = nodes[i + 1];
                Vector2 delta = n2.pos - n1.pos;
                float currentDist = delta.magnitude;
                if (currentDist < 0.001f) continue;

                float difference = (currentDist - segmentLength) / currentDist;
                Vector2 correction = delta * 0.5f * stiffness * difference;

                // 首尾节点不参与修正 (它们跟着玩家动)
                if (i > 0) n1.pos += correction;
                if (i + 1 < segments - 1) n2.pos -= correction;
            }
        }
    }

    // 计算并施加对玩家的牵引力
    void ApplyPullForceToPlayers()
    {
        Vector2 dir = player2.transform.position - player1.transform.position;
        float currentDist = dir.magnitude;

        // 只有当两人距离超过软管总长度时，才开始施加额外的拉力
        if (currentDist > totalLength)
        {
            // 计算拉伸量
            float stretchAmount = currentDist - totalLength;
            Vector2 pullDir = dir.normalized;
            // 拉力大小与拉伸量成正比
            Vector2 force = pullDir * stretchAmount * pullForceMultiplier;

            // P1 被拉向 P2
            player1.AddGlobalForce(force);
            // P2 被拉向 P1 (反向力)
            player2.AddGlobalForce(-force);
        }
    }

    void UpdateVisuals()
    {
        for (int i = 0; i < segments; i++)
        {
            lr.SetPosition(i, nodes[i].pos);
        }
        // 动态调整粗细，充气方连接处稍微粗一点点 (可选视觉效果)
        // float w1 = Mathf.Lerp(hoseWidth * 0.8f, hoseWidth * 1.2f, player1.GetInflationRatio());
        // float w2 = Mathf.Lerp(hoseWidth * 0.8f, hoseWidth * 1.2f, player2.GetInflationRatio());
        // lr.startWidth = w1; lr.endWidth = w2;
    }
}