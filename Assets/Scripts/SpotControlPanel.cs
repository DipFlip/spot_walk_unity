using TMPro;
using UnityEngine;
using UnityEngine.UI;

[DisallowMultipleComponent]
public sealed class SpotControlPanel : MonoBehaviour
{
    [SerializeField] private SpotRosTcpBridge spot;
    [SerializeField] private Button claimButton;
    [SerializeField] private Button releaseButton;
    [SerializeField] private Button powerOnButton;
    [SerializeField] private Button standButton;
    [SerializeField] private Button sitButton;
    [SerializeField] private TMP_Text statusText;

    private string lastAction = "Use the controls below, then drive with WASD and Q/E.";

    private void Awake()
    {
        if (!Application.isPlaying)
        {
            return;
        }

        if (spot == null)
        {
            spot = FindAnyObjectByType<SpotRosTcpBridge>();
        }

        if (spot == null || claimButton == null || releaseButton == null ||
            powerOnButton == null || standButton == null || sitButton == null || statusText == null)
        {
            Debug.LogError("Spot control panel has missing serialized references.", this);
            enabled = false;
            return;
        }

        WireButton(claimButton, TakeLease);
        WireButton(releaseButton, ReleaseLease);
        WireButton(powerOnButton, PowerOn);
        WireButton(standButton, Stand);
        WireButton(sitButton, Sit);
        Refresh();
    }

    private void Update()
    {
        if (Application.isPlaying)
        {
            Refresh();
        }
    }

    private static void WireButton(Button button, UnityEngine.Events.UnityAction action)
    {
        button.onClick.RemoveAllListeners();
        button.onClick.AddListener(action);
    }

    private void TakeLease()
    {
        spot.ClaimLease(out lastAction);
    }

    private void ReleaseLease()
    {
        spot.ReleaseLease(out lastAction);
    }

    private void PowerOn()
    {
        spot.PowerOn(out lastAction);
    }

    private void Stand()
    {
        spot.Stand(out lastAction);
    }

    private void Sit()
    {
        spot.Sit(out lastAction);
    }

    private void Refresh()
    {
        claimButton.interactable = !spot.HasLease;
        releaseButton.interactable = spot.HasLease;
        powerOnButton.interactable = spot.HasLease && !spot.IsPoweredOn;
        standButton.interactable = spot.HasLease && spot.IsPoweredOn && !spot.IsStanding;
        sitButton.interactable = spot.HasLease && spot.IsPoweredOn && spot.IsStanding;

        string connection = spot.IsConnected ? "ROS connected" : "ROS waiting";
        string lease = spot.HasLease ? "leased" : "no lease";
        string power = spot.IsPoweredOn ? "powered" : "off";
        string posture = spot.IsStanding ? "standing" : "sitting";
        statusText.text = $"{connection} | {lease} | {power} | {posture}\n{lastAction}\nDrive: W/S forward, A/D yaw, Q/E strafe";
    }
}
