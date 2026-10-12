from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from flying_aces.core.contracts import Wind


# ---------------------------------------------------------------------------
# Discharge models
# ---------------------------------------------------------------------------

class BatteryDischargeModel(ABC):
    """
    Defines how a battery consumes energy during flight.

    Models receive the flight conditions they need explicitly. This keeps
    battery behavior independent of a particular vehicle implementation.
    """

    @abstractmethod
    def estimate_energy(
        self,
        distance: float,
        payload_weight: float = 0.0,
        wind: Optional["Wind"] = None,
        cruise_speed: float = 15.0,
    ) -> float:
        """
        Estimate energy required for a flight leg.

        Returns:
            Energy in watt-hours (Wh).
        """
        raise NotImplementedError

    @abstractmethod
    def power_draw(
        self,
        payload_weight: float = 0.0,
        wind: Optional["Wind"] = None,
        cruise_speed: float = 15.0,
    ) -> float:
        """
        Estimate instantaneous power draw.

        Returns:
            Power in watts (W).
        """
        raise NotImplementedError


class ConstantPowerDischarge(BatteryDischargeModel):
    """
    Simple model with constant power consumption.

    Useful as a baseline model and for testing the simulator without
    introducing payload or wind effects.
    """

    def __init__(self, power_w: float = 180.0):
        self.power_w = power_w

    def power_draw(
        self,
        payload_weight: float = 0.0,
        wind: Optional["Wind"] = None,
        cruise_speed: float = 15.0,
    ) -> float:
        return self.power_w

    def estimate_energy(
        self,
        distance: float,
        payload_weight: float = 0.0,
        wind: Optional["Wind"] = None,
        cruise_speed: float = 15.0,
    ) -> float:
        if cruise_speed <= 0.0:
            return 0.0

        flight_time_hours = distance / cruise_speed / 3600.0
        return self.power_draw(payload_weight, wind, cruise_speed) * flight_time_hours


class PayloadWindDischarge(BatteryDischargeModel):
    """
    Simple flight-energy model incorporating payload and wind.

    Energy consists of:
        base energy
        + payload penalty
        + wind penalty

    The model is intentionally simple. Its purpose is to provide a useful
    abstraction that can later be replaced by a more physically realistic
    model without changing the Battery class.
    """

    def __init__(
        self,
        base_power_w: float = 180.0,
        payload_power_per_kg: float = 20.0,
        wind_power_per_mps: float = 15.0,
    ):
        self.base_power_w = base_power_w
        self.payload_power_per_kg = payload_power_per_kg
        self.wind_power_per_mps = wind_power_per_mps

    def power_draw(
        self,
        payload_weight: float = 0.0,
        wind: Optional["Wind"] = None,
        cruise_speed: float = 15.0,
    ) -> float:
        wind_speed = wind.speed if wind is not None else 0.0

        return (
            self.base_power_w
            + payload_weight * self.payload_power_per_kg
            + wind_speed * self.wind_power_per_mps
        )

    def estimate_energy(
        self,
        distance: float,
        payload_weight: float = 0.0,
        wind: Optional["Wind"] = None,
        cruise_speed: float = 15.0,
    ) -> float:
        if cruise_speed <= 0.0:
            return 0.0

        flight_time_hours = distance / cruise_speed / 3600.0
        return self.power_draw(
            payload_weight,
            wind,
            cruise_speed,
        ) * flight_time_hours


# ---------------------------------------------------------------------------
# Recharge models
# ---------------------------------------------------------------------------

class BatteryRechargeModel(ABC):
    """
    Defines how quickly a battery can be recharged.
    """

    @abstractmethod
    def recharge_energy(self, dt: float) -> float:
        """
        Return energy added during dt seconds.

        Returns:
            Energy in watt-hours (Wh).
        """
        raise NotImplementedError


class ConstantRecharge(BatteryRechargeModel):
    """
    Recharge at a constant charging power.
    """

    def __init__(self, power_w: float = 500.0):
        self.power_w = power_w

    def recharge_energy(self, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        return self.power_w * dt / 3600.0


# ---------------------------------------------------------------------------
# Battery
# ---------------------------------------------------------------------------

class Battery:
    """
    Concrete battery implementation.

    Battery owns:
        - capacity
        - remaining energy
        - discharge behavior
        - recharge behavior

    Discharge and recharge behavior can be independently swapped.
    """

    def __init__(
        self,
        capacity_wh: float,
        discharge_model: BatteryDischargeModel,
        recharge_model: BatteryRechargeModel,
        initial_percent: float = 100.0,
    ):
        if capacity_wh <= 0.0:
            raise ValueError("Battery capacity must be greater than zero.")

        if not 0.0 <= initial_percent <= 100.0:
            raise ValueError("Initial battery percentage must be between 0 and 100.")

        self.capacity_wh = capacity_wh
        self.discharge_model = discharge_model
        self.recharge_model = recharge_model

        self.remaining_wh = capacity_wh * (initial_percent / 100.0)

    # -----------------------------------------------------------------------
    # State
    # -----------------------------------------------------------------------

    def remaining_energy(self) -> float:
        """Return remaining energy in Wh."""
        return self.remaining_wh

    def remaining_percent(self) -> float:
        """Return remaining battery as a percentage."""
        return (self.remaining_wh / self.capacity_wh) * 100.0

    # -----------------------------------------------------------------------
    # Discharge
    # -----------------------------------------------------------------------

    def discharge(
        self,
        dt: float,
        payload_weight: float = 0.0,
        wind: Optional["Wind"] = None,
        cruise_speed: float = 15.0,
    ) -> float:
        """
        Consume energy for dt seconds.

        Returns:
            Actual energy consumed in Wh.
        """

        if dt <= 0.0 or self.remaining_wh <= 0.0:
            return 0.0

        power_w = self.discharge_model.power_draw(
            payload_weight=payload_weight,
            wind=wind,
            cruise_speed=cruise_speed,
        )

        energy_wh = power_w * dt / 3600.0
        energy_wh = min(energy_wh, self.remaining_wh)

        self.remaining_wh -= energy_wh
        self.remaining_wh = max(0.0, self.remaining_wh)

        return energy_wh

    def discharge_power(self, dt: float, power_w: float) -> float:
        """Consume energy using an externally estimated electrical power (watts)."""
        if dt <= 0 or self.remaining_wh <= 0:
            return 0.0
        if power_w < 0:
            raise ValueError("power_w cannot be negative")
        consumed = min(power_w * dt / 3600.0, self.remaining_wh)
        self.remaining_wh -= consumed
        return consumed

    def estimate_energy(
        self,
        distance: float,
        payload_weight: float = 0.0,
        wind: Optional["Wind"] = None,
        cruise_speed: float = 15.0,
    ) -> float:
        """
        Estimate energy required to travel a distance.

        Returns:
            Estimated energy in Wh.
        """

        return self.discharge_model.estimate_energy(
            distance=distance,
            payload_weight=payload_weight,
            wind=wind,
            cruise_speed=cruise_speed,
        )

    # -----------------------------------------------------------------------
    # Recharge
    # -----------------------------------------------------------------------

    def recharge(self, dt: float) -> float:
        """
        Recharge for dt seconds.

        The recharge model determines how much energy is added.

        Returns:
            Actual energy added in Wh.
        """

        if dt <= 0.0 or self.remaining_wh >= self.capacity_wh:
            return 0.0

        energy_wh = self.recharge_model.recharge_energy(dt)
        energy_wh = min(
            energy_wh,
            self.capacity_wh - self.remaining_wh,
        )

        self.remaining_wh += energy_wh
        self.remaining_wh = min(self.capacity_wh, self.remaining_wh)

        return energy_wh

    # -----------------------------------------------------------------------
    # Lifecycle
    # -----------------------------------------------------------------------

    def reset(self) -> None:
        """Restore the battery to 100%."""
        self.remaining_wh = self.capacity_wh

    def set_percent(self, percent: float) -> None:
        """
        Set battery charge directly.

        Primarily useful for initialization, testing, or debugging.
        Normal simulation code should use discharge() and recharge().
        """

        if not 0.0 <= percent <= 100.0:
            raise ValueError("Battery percentage must be between 0 and 100.")

        self.remaining_wh = self.capacity_wh * (percent / 100.0)

    def is_empty(self) -> bool:
        return self.remaining_wh <= 0.0

    def is_full(self) -> bool:
        return self.remaining_wh >= self.capacity_wh