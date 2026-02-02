using UnityEngine;

/// <summary>
/// 物理驱动的箱子脚本 - 使用自定义Verlet积分实现平移与旋转
/// 解决了穿透墙体、落地不归平以及被玩家撞击旋转的问题
/// </summary>
public class SimpleBox : MonoBehaviour
{
    [Header("基础物理参数")]
    public float mass = 15.0f;           // 质量：越大越难被猪猪推开
    public Vector2 size = new Vector2(1.5f, 1.5f);
    public Vector2 gravity = new Vector2(0, -35.0f);

    [Header("阻尼与摩擦")]
    [Range(0f, 1f)]
    public float linearDamping = 0.95f;  // 平移阻尼（空气阻力/地面摩擦）
    [Range(0f, 1f)]
    public float angularDamping = 0.94f; // 旋转阻尼：越低旋转停得越快
    [Range(0f, 1f)]
    public float friction = 0.2f;        // 与玩家接触时的摩擦力

    [Header("归平与稳定性")]
    [Tooltip("归平速度：让箱子倾向于回到水平或垂直状态")]
    public float uprightSpeed = 3.0f;
    [Tooltip("碰撞旋转系数：角先着地时产生翻转力的强度")]
    public float collisionTorqueFactor = 2.5f;

    [HideInInspector] public Vector2 position;
    [HideInInspector] public float angle; // 弧度

    private Vector2 prevPos;
    private float prevAngle;
    private float inertia;               // 转动惯量
    private SimpleWall[] allWalls;

    void Start()
    {
        position = transform.position;
        prevPos = position;
        angle = transform.rotation.eulerAngles.z * Mathf.Deg2Rad;
        prevAngle = angle;
        allWalls = FindObjectsOfType<SimpleWall>();

        // 矩形转动惯量公式：I = m * (w^2 + h^2) / 12
        inertia = mass * (size.x * size.x + size.y * size.y) / 12f;
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // 1. Verlet 积分：计算平移和旋转
        Vector2 velocity = (position - prevPos) * linearDamping;
        float angularVelocity = (angle - prevAngle) * angularDamping;

        prevPos = position;
        prevAngle = angle;

        position += velocity + gravity * dt * dt;
        angle += angularVelocity;

        // 2. 改进的自动归平逻辑
        // 只有当旋转速度不是特别快的时候，才尝试归平，防止与剧烈碰撞冲突
        if (Mathf.Abs(angularVelocity) < 0.2f)
        {
            float targetAngle = Mathf.Round(angle / (Mathf.PI / 2f)) * (Mathf.PI / 2f);
            float angleDiff = targetAngle - angle;

            // 使用更柔和的插值，而不是直接加法
            // 这样它更像是一个微弱的弹簧力，而不是硬性的坐标修改
            float correction = angleDiff * uprightSpeed * dt;
            angle += correction;

            // 关键：同步更新 prevAngle 抵消掉由于修正产生的虚假角速度
            // 这能防止“自我修正”被 Verlet 误认为是一个真实的物理推力
            prevAngle += correction;
        }

        // 3. 环境碰撞处理 (SAT算法)
        ResolveEnvironmentCollisions();

        ResolveEnvironmentCollisions(); // 处理墙壁
        ResolveSeesawCollisions();      // 新增：处理与跷跷板的碰撞？

        // 4. 更新视觉表现
        transform.position = position;
        transform.rotation = Quaternion.Euler(0, 0, angle * Mathf.Rad2Deg);
    }

    /// <summary>
    /// 被猪猪撞击时调用的方法（由 SlimeSoftBody 调用）
    /// </summary>
    public void AddImpulseAtPosition(Vector2 impulse, Vector2 worldPos)
    {
        // F = ma 导致平移
        position += impulse / mass;

        // 力矩 T = r x F 导致旋转
        Vector2 r = worldPos - position;
        float torque = r.x * impulse.y - r.y * impulse.x;
        angle += torque / inertia;
    }
    [Header("进阶稳定性")]
    [Range(0f, 1f)]
    public float bounciness = 0.2f;       // 弹性系数：0不弹，1全弹。建议设为0.1-0.3
    public float sleepThreshold = 0.05f;  // 静止阈值：速度低于此值时强制停止

    private void ResolveEnvironmentCollisions()
    {
        Vector2[] boxCorners = GetWorldCorners();

        foreach (var wall in allWalls)
        {
            Bounds wB = wall.GetBounds();

            // 定义检测轴：墙的两个轴(法线)和箱子的两个轴(法线)
            Vector2[] axes = new Vector2[4];
            axes[0] = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)); // 箱子局部X轴
            axes[1] = new Vector2(-Mathf.Sin(angle), Mathf.Cos(angle)); // 箱子局部Y轴
            axes[2] = Vector2.right; // 世界坐标X轴
            axes[3] = Vector2.up;    // 世界坐标Y轴

            float minOverlap = float.MaxValue;
            Vector2 smallestAxis = Vector2.zero;

            // SAT检测
            bool isIntersecting = true;
            foreach (Vector2 axis in axes)
            {
                if (!CheckAxisOverlap(axis, boxCorners, wB, out float overlap))
                {
                    isIntersecting = false;
                    break;
                }
                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    smallestAxis = axis;
                }
            }

            if (isIntersecting && minOverlap > 0.005f)
            {
                // 1. 基础挤出逻辑
                Vector2 dirToBox = position - (Vector2)wB.center;
                if (Vector2.Dot(smallestAxis, dirToBox) < 0) smallestAxis = -smallestAxis;
                Vector2 separation = smallestAxis * minOverlap;
                position += separation;

                // 2. 旋转反馈 (保持之前的逻辑)
                Vector2 contactPoint = boxCorners[0];
                float deepestInWall = float.MinValue;
                foreach (var c in boxCorners)
                {
                    float depth = Vector2.Dot((Vector2)wB.center - c, smallestAxis);
                    if (depth > deepestInWall) { deepestInWall = depth; contactPoint = c; }
                }
                Vector2 r = contactPoint - position;
                float torque = (r.x * separation.y - r.y * separation.x) * collisionTorqueFactor;
                angle += torque;

                // 3. 核心改进：修复反复弹跳的速度反馈
                Vector2 currentVel = (position - prevPos);
                float velInNormal = Vector2.Dot(currentVel, smallestAxis);

                if (velInNormal < 0) // 只有当物体正在向墙内运动时才处理反弹
                {
                    // 分解速度：法向速度（弹起）和切向速度（滑动摩擦）
                    Vector2 normalVel = smallestAxis * velInNormal;
                    Vector2 tangentVel = currentVel - normalVel;

                    // 应用弹性系数 (bounciness) 和 摩擦损耗
                    // 注意：这里使用 -bounciness 来反转方向，且值必须小于 1.0
                    currentVel = tangentVel * 0.8f - normalVel * bounciness;

                    // 静止判定：如果反弹后的速度非常小，直接清零，防止微小跳动
                    if (currentVel.magnitude < sleepThreshold)
                    {
                        currentVel = Vector2.zero;
                    }

                    // 更新 prevPos 来重新设定速度
                    prevPos = position - currentVel;
                }

                // 旋转摩擦增强
                prevAngle = Mathf.Lerp(prevAngle, angle, 0.2f);
            }
        }
    }

    private bool CheckAxisOverlap(Vector2 axis, Vector2[] boxCorners, Bounds wallBounds, out float overlap)
    {
        overlap = 0;
        // 计算箱子在轴上的投影
        float minA = float.MaxValue, maxA = float.MinValue;
        foreach (var p in boxCorners)
        {
            float proj = Vector2.Dot(p, axis);
            minA = Mathf.Min(minA, proj);
            maxA = Mathf.Max(maxA, proj);
        }

        // 计算墙体在轴上的投影
        Vector2[] wallCorners = {
            new Vector2(wallBounds.min.x, wallBounds.min.y),
            new Vector2(wallBounds.max.x, wallBounds.min.y),
            new Vector2(wallBounds.max.x, wallBounds.max.y),
            new Vector2(wallBounds.min.x, wallBounds.max.y)
        };
        float minB = float.MaxValue, maxB = float.MinValue;
        foreach (var p in wallCorners)
        {
            float proj = Vector2.Dot(p, axis);
            minB = Mathf.Min(minB, proj);
            maxB = Mathf.Max(maxB, proj);
        }

        if (maxA < minB || maxB < minA) return false;

        overlap = Mathf.Min(maxA, maxB) - Mathf.Max(minA, minB);
        return true;
    }

    public Vector2[] GetWorldCorners()
    {
        Vector2[] corners = new Vector2[4];
        Vector2 halfSize = size * 0.5f;
        corners[0] = RotatePoint(new Vector2(-halfSize.x, -halfSize.y));
        corners[1] = RotatePoint(new Vector2(halfSize.x, -halfSize.y));
        corners[2] = RotatePoint(new Vector2(halfSize.x, halfSize.y));
        corners[3] = RotatePoint(new Vector2(-halfSize.x, halfSize.y));
        return corners;
    }

    private Vector2 RotatePoint(Vector2 localPoint)
    {
        float cos = Mathf.Cos(angle);
        float sin = Mathf.Sin(angle);
        return new Vector2(
            position.x + localPoint.x * cos - localPoint.y * sin,
            position.y + localPoint.x * sin + localPoint.y * cos
        );
    }

    public Bounds GetBounds() => new Bounds(position, size);

    void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        // 绘制带旋转的线框
        Vector2[] corners = GetWorldCorners();
        for (int i = 0; i < 4; i++)
            Gizmos.DrawLine(corners[i], corners[(i + 1) % 4]);
    }

    private void ResolveSeesawCollisions()
    {
        SimpleSeesaw[] allSeesaws = FindObjectsOfType<SimpleSeesaw>();
        foreach (var seesaw in allSeesaws)
        {
            Vector2[] boxCorners = GetWorldCorners();
            Vector2[] seesawCorners = seesaw.GetWorldCorners();

            // SAT 需要检测的轴：箱子的2个局部轴 + 跷跷板的2个局部轴
            Vector2[] axes = new Vector2[4];
            axes[0] = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));
            axes[1] = new Vector2(-Mathf.Sin(angle), Mathf.Cos(angle));
            axes[2] = new Vector2(Mathf.Cos(seesaw.angle), Mathf.Sin(seesaw.angle));
            axes[3] = new Vector2(-Mathf.Sin(seesaw.angle), Mathf.Cos(seesaw.angle));

            float minOverlap = float.MaxValue;
            Vector2 smallestAxis = Vector2.zero;

            // 1. SAT 检测
            bool isIntersecting = true;
            foreach (Vector2 axis in axes)
            {
                if (!CheckOBBOverlap(axis, boxCorners, seesawCorners, out float overlap))
                {
                    isIntersecting = false;
                    break;
                }
                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    smallestAxis = axis;
                }
            }

            // 2. 碰撞响应
            if (isIntersecting)
            {
                // 确保挤出方向是从跷跷板指向箱子
                Vector2 dir = position - (Vector2)seesaw.transform.position;
                if (Vector2.Dot(smallestAxis, dir) < 0) smallestAxis = -smallestAxis;

                Vector2 separation = smallestAxis * minOverlap;

                // --- 对箱子的影响 ---
                position += separation; // 挤出位移

                // 找到接触点（取箱子进入跷跷板最深的顶点）
                Vector2 contactPoint = boxCorners[0];
                float maxDepth = float.MinValue;
                foreach (var c in boxCorners)
                {
                    float d = Vector2.Dot((Vector2)seesaw.transform.position - c, smallestAxis);
                    if (d > maxDepth) { maxDepth = d; contactPoint = c; }
                }

                // 应用力矩给箱子，让它根据碰撞点旋转
                Vector2 rBox = contactPoint - position;
                float boxTorque = (rBox.x * separation.y - rBox.y * separation.x) * collisionTorqueFactor;
                angle += boxTorque;

                // --- 对跷跷板的影响（关键：压住跷跷板） ---
                // 我们给跷跷板施加一个反向的冲量。
                // 冲量大小取决于挤出位移的大小，模拟箱子的重量
                Vector2 impulseToSeesaw = -separation * (mass * 0.1f); // 0.1f是传导系数，可调
                seesaw.AddImpulseAtPosition(impulseToSeesaw, contactPoint);

                // 速度修正，防止穿透后产生过大的动能
                prevPos = position - (position - prevPos) * 0.8f;
                prevAngle = Mathf.Lerp(prevAngle, angle, 0.2f);
            }
        }
    }

    // 辅助方法：计算两个多边形在轴上的投影重叠
    private bool CheckOBBOverlap(Vector2 axis, Vector2[] cornersA, Vector2[] cornersB, out float overlap)
    {
        overlap = 0;
        float minA = float.MaxValue, maxA = float.MinValue;
        foreach (var p in cornersA)
        {
            float proj = Vector2.Dot(p, axis);
            minA = Mathf.Min(minA, proj);
            maxA = Mathf.Max(maxA, proj);
        }

        float minB = float.MaxValue, maxB = float.MinValue;
        foreach (var p in cornersB)
        {
            float proj = Vector2.Dot(p, axis);
            minB = Mathf.Min(minB, proj);
            maxB = Mathf.Max(maxB, proj);
        }

        if (maxA < minB || maxB < minA) return false;

        overlap = Mathf.Min(maxA, maxB) - Mathf.Max(minA, minB);
        return true;
    }
}