import picarx_improved

if __name__ == "__main__":
    px = picarx_improved.Picarx()
    while True:
        print("\nAvailable maneuvers:")
        print("1. Forward/Backward")
        print("2. Parallel Parking")
        print("3. K-turn")
        print("Type 'exit' to quit")

        choice = input("Enter your choice: ")

        if choice.lower() == 'exit':
            break
        elif choice == '1':
            direction = int(input("Enter direction (0 for forward, 1 for backward): "))
            angle = int(input("Enter angle: "))
            speed = int(input("Enter speed: "))
            duration = float(input("Enter duration: "))
            px.f_b(direction, angle, speed, duration)
        elif choice == '2':
            direction = int(input("Enter direction for parking (0 for right, 1 for left): "))
            px.parallelParking(direction)
        elif choice == '3':
            duration = float(input("Enter duration of stop for K-turn: "))
            px.Kturn(duration)
        else:
            print("Invalid choice. Please try again.")