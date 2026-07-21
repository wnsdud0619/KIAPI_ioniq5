import psycopg2
from psycopg2 import pool

connection_pool = pool.SimpleConnectionPool(
    1, 10,
    host="218.157.234.17",
    port="5432",
    database="kiapidb",
    user="remote_user",
    password="remote78&*"
)

def _execute(query, params=None, fetch=False):
    conn = connection_pool.getconn()

    try:
        with conn.cursor() as cur:
            cur.execute(query, params)

            if fetch:
                return cur.fetchall()
            else:
                conn.commit()
                return cur.rowcount

    except Exception as e:
        conn.rollback()
        print("DB 오류:", e)
        raise

    finally:
        connection_pool.putconn(conn)

def select(query, params=None):
    """
    SELECT 전용
    return: 조회 결과 (list of tuple)
    """
    return _execute(query, params, fetch=True)


def insert(query, params=None):
    """
    INSERT 전용
    return: 영향받은 행 수
    """
    return _execute(query, params)


def update(query, params=None):
    """
    UPDATE 전용
    return: 영향받은 행 수
    """
    return _execute(query, params)


def delete(query, params=None):
    """
    DELETE 전용
    return: 영향받은 행 수
    """
    return _execute(query, params)