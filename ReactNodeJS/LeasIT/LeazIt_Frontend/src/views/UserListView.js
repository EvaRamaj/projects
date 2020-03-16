"use strict";

import React from 'react';

import { UserList } from '../components/UserList';

import UserService from '../services/UserService';


export class UserListView extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: false,
            data: []
        };
        this.deleteUser = this.deleteUser.bind(this);
    }

    componentWillMount(){
        this.setState({
            loading: true
        });


        UserService.getUsers().then((data) => {
            this.setState({
                data: [...data],
                loading: false,
                lessor_req: false
            });
        }).catch((e) => {
            console.error(e);
        });
    }

    deleteUser(id) {
        this.setState({
            data: [...this.state.data],
            loading: true
        });

        UserService.deleteUser(id).then((message) => {

            let userIndex = this.state.data.map(user => user['_id']).indexOf(id);
            let users = this.state.data;
            users.splice(userIndex, 1);
            this.setState({
                data: [...users],
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (
            <UserList data={this.state.data} req={this.state.lessor_req} onDelete={(id) => this.deleteUser(id)}/>
        );
    }
}