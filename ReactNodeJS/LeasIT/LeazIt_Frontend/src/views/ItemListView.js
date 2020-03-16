"use strict";

import React from 'react';

import { ItemList } from '../components/ItemList';
import { MyItemList } from '../components/MyItemList';

import ItemService from '../services/ItemService';
import UserEvaluationService from '../services/UserEvaluationService';
const rowSize =3;

export class ItemListView extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: false,
            full_rows: 0,
            extra_cols: 0,
            data: []
        };
    }

    componentWillMount(){
        this.setState({
            loading: true
        });
        if(this.props.match.path === '/items') {
            // let full_rows = Math.floor(data.length/rowSize);
            // let extra_cols = data.length - full_rows*rowSize;
            ItemService.getItems().then((data) => {
                this.setState({
                    data: [...data],
                    full_rows:Math.floor(data.length/rowSize),
                    extra_cols:data.length - Math.floor(data.length/rowSize)*rowSize,
                    loading: false
                });
            }).catch((e) => {
                console.error(e);
            });
        }
        else{
            ItemService.getMyItems().then((data) => {
                let full_rows = Math.floor(data.length/rowSize);
                let extra_cols = data.length - full_rows*rowSize;
                UserEvaluationService.getUserEvaluations().then((user_evaluations) => {
                    console.log(user_evaluations);
                    this.setState({
                        data: [...data],
                        full_rows:full_rows,
                        extra_cols:extra_cols,
                        user_evals: user_evaluations,
                        loading: false
                    });
                    console.log(this.state)
                }).catch((e) => {
                    console.error(e);
                });
            }).catch((e) => {
                console.log(e);
            });
        }
    }


    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }
        if(this.props.match.path === '/items') {
            return (
                    <ItemList user_evals={this.state.user_evals} data={this.state.data} full_rows = {this.state.full_rows} extra_cols={this.state.extra_cols}/>
            );
        }
        else{
            return (
                    <MyItemList  user_evals = {this.state.user_evals} data={this.state.data} full_rows = {this.state.full_rows} extra_cols = {this.state.extra_cols}/>
            );

        }
    }
}
