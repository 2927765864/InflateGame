using UnityEngine;

public class SlimeManager : MonoBehaviour
{
    [Header("玩家引用")]
    public SlimeSoftBody player1;
    public SlimeSoftBody player2;

    [Header("气体总量控制")]
    [Tooltip("两个玩家的目标面积之和 (固定总量)")]
    public float totalTargetArea = 10f;

    [Tooltip("充气转移速度 (每秒转移多少面积)")]
    public float transferSpeed = 5f;

    void Start()
    {
        // 游戏开始时，平分气体
        if (player1 != null && player2 != null)
        {
            player1.targetArea = totalTargetArea / 2f;
            player2.targetArea = totalTargetArea / 2f;
        }
    }

    void Update()
    {
        if (player1 == null || player2 == null) return;

        float transferAmount = 0;

        // --- 逻辑：按住充气键才会夺取对方气体，松开则保持现状 ---

        // 玩家 1 (键盘 I 键) 充气：从 P2 夺取气体
        if (Input.GetKey(KeyCode.I))
        {
            transferAmount = transferSpeed * Time.deltaTime;
        }
        // 玩家 2 (手柄 X 键) 充气：从 P1 夺取气体
        else if (Input.GetKey(KeyCode.JoystickButton2))
        {
            transferAmount = -transferSpeed * Time.deltaTime;
        }

        if (Mathf.Abs(transferAmount) > 0)
        {
            ApplyGasTransfer(transferAmount);
        }
    }

    void ApplyGasTransfer(float amount)
    {
        float newP1Area = player1.targetArea + amount;
        float newP2Area = player2.targetArea - amount;

        // 边界安全检查：确保任何一方都不会低于其最小面积限制
        if (newP1Area >= player1.minArea && newP2Area >= player2.minArea)
        {
            player1.targetArea = newP1Area;
            player2.targetArea = newP2Area;
        }
    }
}