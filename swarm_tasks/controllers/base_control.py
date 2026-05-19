import swarm_tasks.controllers.potential_field as potf


def base_control(
    i,
    bot,
    goal,
    field_weights={"bots": 100, "obstacles": 1, "borders": 1, "goal": -3, "items": 0},
    order=2,
):
    """
    For keeping the overall trajectory smooth and away from obstacles
    Args:
            - bot
            - field_weights
            - order: Inverse square law by default. Low order recommended
    Returns: Cmd object (direction & speed)
    """

    cmd = potf.get_field(
        i,
        bot.get_position(),
        goal,
        bot.sim,
        weights=field_weights,
        order=order,
        max_dist=3,
    )
    return cmd


def obstacle_avoidance(
    i,
    bot,
    goal,
    field_weights={
        "bots": 100,
        "obstacles": 1,
        "borders": 0.5,
        "goal": -3,
        "items": 0.05,
    },
    order=4,
    k=3,
):
    """
    For close range obstacle avoidance
    Args:
            - bot
            - field_weights
            - order: High order recommended
            - k: Multiplier
    Returns: Cmd object (direction & speed)
    """
    cmd = potf.get_field(
        i,
        bot.get_position(),
        goal,
        bot.sim,
        weights=field_weights,
        order=order,
        max_dist=bot.size + 0.4,
        item_types=["all"],
    )

    return cmd * k


def exp_control(
    bot,
    field_weights={"bots": 100, "obstacles": 1, "borders": 1, "goal": -3, "items": 0},
    order=2,
):
    """
    For keeping the overall trajectory smooth and away from obstacles
    Args:
            - bot
            - field_weights
            - order: Inverse square law by default. Low order recommended
    Returns: Cmd object (direction & speed)
    """
    cmd = potf.exp_field(
        bot.get_position(), bot.sim, weights=field_weights, order=order, max_dist=3
    )

    return cmd


def exp_obstacle_avoidance(
    bot,
    field_weights={
        "bots": 100,
        "obstacles": 1,
        "borders": 0.5,
        "goal": -3,
        "items": 0.05,
    },
    order=4,
    k=3,
):
    """
    For close range obstacle avoidance
    Args:
            - bot
            - field_weights
            - order: High order recommended
            - k: Multiplier
    Returns: Cmd object (direction & speed)
    """
    cmd = potf.exp_field(
        bot.get_position(),
        bot.sim,
        weights=field_weights,
        order=order,
        max_dist=bot.size + 0.4,
        item_types=["all"],
    )

    return cmd * k
